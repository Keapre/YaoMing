import cv2
import numpy as np
import time


# green
COLOR_A_LOWER = np.array([59, 149, 49])
COLOR_A_UPPER = np.array([76, 255, 128])

# purple
COLOR_B_LOWER = np.array([136, 39, 45])
COLOR_B_UPPER = np.array([175, 198, 120])

# --- Hysteresis thresholding (kept from before - unrelated to motion prediction) ---
STRICT_MARGIN_H = 4
STRICT_MARGIN_S = 20
STRICT_MARGIN_V = 15

# --- Pre-threshold smoothing (kept from before) ---
GAUSSIAN_BLUR_KERNEL = 5

MIN_AREA = 450
MORPH_KERNEL = 7

# --- Zones ---
ZONE_LINES = [0.25, 0.50, 0.75]

# --- Zone-switch hysteresis (prevents flicker between two similarly-loaded zones) ---
ZONE_SWITCH_MIN_MARGIN = 1   # a new zone must have at least this many MORE balls than the
                            # currently locked zone before we switch to it

# --- Steering smoothing (applied to tx within the winning zone) ---
SMOOTHING_ALPHA = 0.15
LOST_TARGET_FRAMES = 5

HORIZONTAL_FOV_DEG = 54.5
VERTICAL_FOV_DEG = 41.0

# --- Ball tracking (pixel space, px/s - needed to compute per-ball and
#     per-zone average velocity, sent to the robot controller) ---
TRACK_MATCH_DIST_PX = 60     # max pixel distance to match a detection to an existing track
                              # between frames
MAX_MISSED_FRAMES = 5         # keep a track alive this many frames even if not currently detected
VELOCITY_ALPHA = 0.3          # EMA smoothing on velocity (px/s). Lower = smoother, more lag.


_smoothed_tx = 0.0
_lost_counter = 0
_locked_zone = None
_locked_zone_count = 0

_tracks = []           # list of dicts: cx, cy, vx, vy (px/s), area, color, contour, missed
_last_frame_time = None
_next_track_id = 0


def match_and_update_tracks(detections, dt):
    """detections: list of (cx, cy, area, color, contour), pixel space.
    Matches each existing track to the nearest unmatched detection within
    TRACK_MATCH_DIST_PX, updates position + EMA-smoothed velocity (px/s).
    Unmatched detections spawn new tracks. Tracks missed too long are dropped."""
    global _tracks, _next_track_id

    unmatched = list(detections)
    safe_dt = max(dt, 1e-3)

    for track in _tracks:
        if unmatched:
            candidate = min(unmatched, key=lambda d: np.hypot(d[0] - track['cx'], d[1] - track['cy']))
            dist = np.hypot(candidate[0] - track['cx'], candidate[1] - track['cy'])
            if dist < TRACK_MATCH_DIST_PX:
                raw_vx = (candidate[0] - track['cx']) / safe_dt
                raw_vy = (candidate[1] - track['cy']) / safe_dt
                track['vx'] = VELOCITY_ALPHA * raw_vx + (1 - VELOCITY_ALPHA) * track['vx']
                track['vy'] = VELOCITY_ALPHA * raw_vy + (1 - VELOCITY_ALPHA) * track['vy']
                track['cx'], track['cy'] = candidate[0], candidate[1]
                track['area'] = candidate[2]
                track['color'] = candidate[3]
                track['contour'] = candidate[4]
                track['missed'] = 0
                unmatched.remove(candidate)
                continue
        track['missed'] += 1

    for d in unmatched:
        _tracks.append({
            'id': _next_track_id, 'cx': d[0], 'cy': d[1],
            'vx': 0.0, 'vy': 0.0, 'area': d[2], 'color': d[3],
            'contour': d[4], 'missed': 0,
        })
        _next_track_id += 1

    _tracks = [t for t in _tracks if t['missed'] <= MAX_MISSED_FRAMES]


def _strict_bounds(lower, upper):
    margin = np.array([STRICT_MARGIN_H, STRICT_MARGIN_S, STRICT_MARGIN_V])
    strict_lower = lower + margin
    strict_upper = upper - margin
    strict_lower = np.minimum(strict_lower, strict_upper)
    return strict_lower, strict_upper


def get_mask(hsv, lower, upper):
    strict_lower, strict_upper = _strict_bounds(lower, upper)
    loose_mask = cv2.inRange(hsv, lower, upper)
    strict_mask = cv2.inRange(hsv, strict_lower, strict_upper)

    num_labels, labels = cv2.connectedComponents(loose_mask)
    if num_labels <= 1:
        final_mask = loose_mask
    else:
        strict_labels = np.unique(labels[strict_mask > 0])
        strict_labels = strict_labels[strict_labels != 0]
        keep = np.isin(labels, strict_labels)
        final_mask = np.where(keep, 255, 0).astype(np.uint8)

    kernel = np.ones((MORPH_KERNEL, MORPH_KERNEL), np.uint8)
    final_mask = cv2.erode(final_mask, kernel, iterations=1)
    final_mask = cv2.dilate(final_mask, kernel, iterations=2)
    return final_mask


def pixel_to_tx(px, img_w):
    norm_x = (px - img_w / 2.0) / (img_w / 2.0)
    return norm_x * (HORIZONTAL_FOV_DEG / 2.0)


def runPipeline(image, llrobot):
    global _smoothed_tx, _lost_counter, _locked_zone, _locked_zone_count, _last_frame_time

    img_h, img_w = image.shape[:2]

    now = time.time()
    dt = (now - _last_frame_time) if _last_frame_time is not None else 0.0
    _last_frame_time = now

    # convert fractional line positions to actual pixel x-coordinates for this
    # frame's resolution, then build the full boundary list (frame edges + lines)
    line_px = [int(f * img_w) for f in ZONE_LINES]
    boundaries_px = [0] + line_px + [img_w]
    num_zones = len(boundaries_px) - 1

    blurred = cv2.GaussianBlur(image, (GAUSSIAN_BLUR_KERNEL, GAUSSIAN_BLUR_KERNEL), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask_a = get_mask(hsv, COLOR_A_LOWER, COLOR_A_UPPER)
    mask_b = get_mask(hsv, COLOR_B_LOWER, COLOR_B_UPPER)

    contours_a, _ = cv2.findContours(mask_a, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_b, _ = cv2.findContours(mask_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    tagged = [(c, "A") for c in contours_a if cv2.contourArea(c) >= MIN_AREA]
    tagged += [(c, "B") for c in contours_b if cv2.contourArea(c) >= MIN_AREA]

    detections = []  # (cx, cy, area, color, contour)
    for c, color in tagged:
        area = cv2.contourArea(c)
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        detections.append((cx, cy, area, color, c))

    match_and_update_tracks(detections, dt)

    # bucket every CURRENTLY-VISIBLE tracked ball into a zone - using tracks
    # instead of raw detections gives each ball a velocity (px/s) alongside
    # its position
    zones = [[] for _ in range(num_zones)]  # each entry: list of track dicts
    count_a, count_b = 0, 0
    for t in _tracks:
        if t['missed'] != 0:
            continue
        cx = t['cx']
        zone_idx = 0
        for i in range(num_zones):
            if boundaries_px[i] <= cx < boundaries_px[i + 1]:
                zone_idx = i
                break
        else:
            zone_idx = num_zones - 1
        zones[zone_idx].append(t)
        if t['color'] == "A":
            count_a += 1
        else:
            count_b += 1

    # draw zone divider lines - bright, thick, and labeled with their position
    # so each one is clearly visible and identifiable while tuning
    for i, x in enumerate(line_px):
        cv2.line(image, (x, 0), (x, img_h), (0, 255, 255), 2)
        cv2.putText(image, f"{ZONE_LINES[i]:.2f}", (x + 3, img_h - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

    for i, z in enumerate(zones):
        zx0 = boundaries_px[i]
        zx1 = boundaries_px[i + 1]
        cx_text = int((zx0 + zx1) / 2) - 5
        cv2.putText(image, str(len(z)), (cx_text, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # draw a box + velocity arrow on every currently-tracked ball, color-coded
    for zone in zones:
        for t in zone:
            x, y, w, h = cv2.boundingRect(t['contour'])
            box_color = (0, 255, 0) if t['color'] == "A" else (255, 0, 255)
            cv2.rectangle(image, (x, y), (x + w, y + h), box_color, 2)

            # velocity arrow: direction + magnitude of this ball's estimated
            # motion, scaled down so it stays a reasonable on-screen length
            speed = np.hypot(t['vx'], t['vy'])
            arrow_end = (int(t['cx'] + t['vx'] * 0.2), int(t['cy'] + t['vy'] * 0.2))
            cv2.arrowedLine(image, (int(t['cx']), int(t['cy'])), arrow_end, (0, 255, 255), 2, tipLength=0.3)
            cv2.putText(image, f"{int(speed)}", (int(t['cx']) + 8, int(t['cy']) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

    zone_counts = [len(z) for z in zones]
    best_zone_idx = None
    raw_target_found = False
    bx, by, bw, bh = 0.0, 0.0, 0.0, 0.0
    avg_zone_speed = 0.0

    if any(c > 0 for c in zone_counts):
        center_zone = (num_zones - 1) / 2.0
        # pick the zone with the most balls; ties broken by closeness to center
        best_zone_idx = max(
            range(num_zones),
            key=lambda i: (zone_counts[i], -abs(i - center_zone))
        )

        # hysteresis: only switch away from the currently locked zone if the
        # new candidate has meaningfully more balls in it
        if (_locked_zone is None) or (zone_counts[best_zone_idx] > _locked_zone_count + ZONE_SWITCH_MIN_MARGIN) or (zone_counts[_locked_zone] if _locked_zone is not None else 0) == 0:
            _locked_zone = best_zone_idx
        chosen_zone = _locked_zone
        _locked_zone_count = zone_counts[chosen_zone]

        chosen_balls = zones[chosen_zone]
        if chosen_balls:
            raw_target_found = True
            total_area = sum(t['area'] for t in chosen_balls)
            avg_cx = sum(t['cx'] * t['area'] for t in chosen_balls) / total_area
            avg_cy = sum(t['cy'] * t['area'] for t in chosen_balls) / total_area

            # average ball velocity (px/s) in the currently targeted zone -
            # simple mean speed magnitude across all balls in that zone
            avg_zone_speed = sum(np.hypot(t['vx'], t['vy']) for t in chosen_balls) / len(chosen_balls)

            raw_tx = pixel_to_tx(avg_cx, img_w)
            _smoothed_tx = SMOOTHING_ALPHA * raw_tx + (1 - SMOOTHING_ALPHA) * _smoothed_tx
            _lost_counter = 0

            pts = np.vstack([t['contour'].reshape(-1, 2) for t in chosen_balls])
            bx, by, bw, bh = cv2.boundingRect(pts)
            cv2.rectangle(image, (bx, by), (bx + bw, by + bh), (0, 0, 255), 3)

            # highlight the winning zone's column using its actual (possibly
            # unequal-width) boundaries
            zx0 = boundaries_px[chosen_zone]
            zx1 = boundaries_px[chosen_zone + 1]
            overlay = image.copy()
            cv2.rectangle(overlay, (zx0, 0), (zx1, img_h), (0, 0, 255), -1)
            cv2.addWeighted(overlay, 0.15, image, 0.85, 0, image)

            cv2.circle(image, (int(avg_cx), int(avg_cy)), 6, (0, 0, 255), -1)

    cv2.putText(image, f"zone: {best_zone_idx}  smoothed tx: {round(_smoothed_tx,1)}",
                (5, img_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(image, f"zoneCounts: {zone_counts}  avgSpeed: {round(avg_zone_speed,1)}px/s",
                (5, img_h - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    if not raw_target_found:
        _lost_counter += 1
        if _lost_counter > LOST_TARGET_FRAMES:
            _smoothed_tx = 0.0
            _locked_zone = None
            _locked_zone_count = 0

    has_target = 1.0 if (raw_target_found or _lost_counter <= LOST_TARGET_FRAMES) else 0.0
    largestContour = np.array([[]])

    llrobot[0] = _smoothed_tx
    llrobot[1] = float(_locked_zone) if _locked_zone is not None else -1.0
    llrobot[2] = float(count_a + count_b)
    llrobot[3] = has_target
    llrobot[4] = bx
    llrobot[5] = by
    llrobot[6] = bw
    llrobot[7] = avg_zone_speed   # REPURPOSED from bh: average ball speed (px/s) in the
                                   # currently targeted zone, sent to the robot controller

    return largestContour, image, llrobot