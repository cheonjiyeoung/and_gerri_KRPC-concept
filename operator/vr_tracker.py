import threading
import time
import xr
from pubsub import pub
import glfw
import logging

VR_UPDATE_INTERVAL = 0.1

class VRTracker:
    def __init__(self):
        self.context = None
        self.frame_index = 0
        self.update_interval = VR_UPDATE_INTERVAL
        self.window = None
        self.is_tracking = False
        self.tracking_thread = None
        logging.basicConfig(level=logging.ERROR)
        self.logger = logging.getLogger(__name__)


    def connect(self):
        self.tracking_thread = threading.Thread(target=self.start_tracking, daemon=True)
        self.tracking_thread.start()
        print("[🔄] WebRtcCommandChannel Running in Background...")

    def start_tracking(self, debug=False):
        while True:
            try:
                with (xr.ContextObject(
                        instance_create_info=xr.InstanceCreateInfo(
                            enabled_extension_names=[
                                # A graphics extension is mandatory (without a headless extension)
                                xr.KHR_OPENGL_ENABLE_EXTENSION_NAME,
                            ],
                        ),
                ) as context):
                    self.context = context
                    # Loop over the render frames
                    for frame_index, frame_state in enumerate(context.frame_loop()):
                        view_state, views = xr.locate_views(
                            session=context.session,
                            view_locate_info=xr.ViewLocateInfo(
                                view_configuration_type=context.view_configuration_type,
                                display_time=frame_state.predicted_display_time,
                                space=context.space,
                            )
                        )
                        flags = xr.ViewStateFlags(view_state.view_state_flags)
                        if flags & xr.ViewStateFlags.POSITION_VALID_BIT:
                            view = views[xr.Eye.LEFT]
                            if debug:
                                print(view)
                            try:
                                pub.sendMessage('vr_pose', position=view.pose.position, orientation=view.pose.orientation)
                            except Exception as e:
                                print(e)
                        time.sleep(self.update_interval)

            except xr.exception.FormFactorUnavailableError:
                self.logger.error("VR device is unavailable. Make sure it is plugged in _and_ powered on.")
            except Exception as e:
                self.logger.error(f"Unexpected error: {e}")
            finally:
                self.disconnect()
                time.sleep(5)


    def disconnect(self):
        if self.context is not None:
            try:
                self.context.__exit__(None, None, None)
            except Exception as e:
                self.logger.error(f"Error during context exit: {e}")
            self.context = None
        if self.window:
            glfw.destroy_window(self.window)
            glfw.terminate()
