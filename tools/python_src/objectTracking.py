import sys
import cv2


def main():
    # show help
    if len(sys.argv) < 2:
        print(
            "Usage: tracker.py <video_name>\n"
            "examples:\n"
            "python tracker.py Bolt/img/%04d.jpg\n"
            "python tracker.py faceocc2.webm\n"
        )
        return

    video = sys.argv[1]
    cap = cv2.VideoCapture(video)

    if not cap.isOpened():
        print(f"Error: Could not open video source: {video}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0  # fallback if FPS is unavailable

    # get first frame
    ok, frame = cap.read()
    if not ok or frame is None:
        print("Error: Could not read first frame.")
        return

    # get bounding box
    roi = cv2.selectROI("tracker", frame, fromCenter=False, showCrosshair=True)

    # quit if ROI was not selected
    if roi[2] == 0 or roi[3] == 0:
        return

    # create a tracker object (KCF)
    tracker = cv2.TrackerKCF_create()

    # initialize the tracker
    tracker.init(frame, roi)

    print("Start the tracking process, press ESC to quit.")

    # open output file
    output_path = "tracking_output.txt"
    with open(output_path, "w") as f:
        # optional header
        f.write("# timestamp x y z q1 q2 q3 q4\n")

        frame_idx = 0

        # perform the tracking process
        while True:
            # get frame from the video
            ok, frame = cap.read()

            # stop the program if no more images
            if not ok or frame is None:
                break

            # update the tracking result
            ok, roi = tracker.update(frame)

            # timestamp (seconds)
            timestamp = frame_idx / fps

            # draw the tracked object + write pose
            if ok:
                x, y, w, h = roi
                cx = -(x + w / 2.0)
                cy = (y + h / 2.0)

                # Write: timestamp x y z q1 q2 q3 q4
                # z=0, neutral quaternion=(0,0,0,1)
                f.write(f"{timestamp:.6f} {cx:.3f} {cy:.3f} 0 0 0 0 1\n")

                # draw rectangle
                x_i, y_i, w_i, h_i = map(int, roi)
                cv2.rectangle(frame, (x_i, y_i), (x_i + w_i, y_i + h_i), (255, 0, 0), 2, 1)
            else:
                cv2.putText(
                    frame,
                    "Tracking failure detected",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

            # show image with the tracked object
            cv2.imshow("tracker", frame)

            # quit on ESC button
            if cv2.waitKey(1) == 27:
                break

            frame_idx += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"Tracking data saved to: {output_path}")


if __name__ == "__main__":
    main()