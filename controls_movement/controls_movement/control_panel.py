import cv2
import threading

control_panel_name = "Adjust Thresholds"

def create_control_panel(controls):
    cv2.namedWindow(control_panel_name)
    for control in controls.items():
        name, v = control
        default_value = None
        value, maximum, minimum, smaller_than = [*v, *([default_value] * (4 - len(v)))] # https://stackoverflow.com/a/59303329/7577786
        maximum = maximum if maximum else 255
        minimum = minimum if minimum != None else 0
        # Use default arguments to capture current 'name' value
        def on_change(val, name=name, minimum=minimum):
            if val < minimum:
                val = minimum
            controls[name][0] = val
        cv2.createTrackbar(
            name, 
            control_panel_name, 
            value, 
            maximum, 
            on_change
        )
    input_thread = threading.Thread(target=cv2.waitKey)
    input_thread.daemon = True
    input_thread.start()
