"""
Pan-Tilt & Zoom Command Functions

Common command object structure:
- `topic` (str): Command category.
- `value` (list or float): Payload data.
- `target` (str, optional): Robot ID or "all" for broadcast.
- `option` (dict, optional): Additional parameters (e.g., speed, acceleration, blocking, timeout).
"""

def pan_tilt(pan_tilt_angle: list[float], target: str = "all", **option) -> dict:
    """
    Set absolute pan and tilt angles.

    Parameters:
        pan_tilt_angle (list of float): [pan, tilt] angles in degrees.
        target (str): Robot ID or "all" to broadcast. Defaults to "all".
        **option: Additional parameters such as:
            - speed (float): Angular speed limit in deg/s.
            - acceleration (float): Motion acceleration in deg/s^2.
            - blocking (bool): Wait until motion completes.
            - timeout (float): Max wait time in seconds if blocking is True.

    Returns:
        dict: Command object with keys:
            - 'topic': 'pan_tilt'
            - 'value': pan_tilt_angle
            - 'target': target (if specified)
            - 'option': option (if provided)

    Example:
        >>> cmd = pan_tilt([30.0, -12.0], target="spot_cam", speed=45)
        >>> # {'topic': 'pan_tilt', 'value': [30.0, -12.0], 'target': 'spot_cam', 'option': {'speed': 45}}
    """
    command = {
        'topic': 'pan_tilt',
        'value': pan_tilt_angle
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def pan_tilt_step(pan_tilt_angle_step: list[float], target: str = None, **option) -> dict:
    """
    Adjust pan and tilt by a relative step.

    Parameters:
        pan_tilt_angle_step (list of float): [Δpan, Δtilt] increments in degrees.
        target (str, optional): Robot ID to apply the step. Defaults to None.
        **option: Additional parameters (see pan_tilt documentation).

    Returns:
        dict: Command object with keys:
            - 'topic': 'pan_tilt_step'
            - 'value': pan_tilt_angle_step
            - 'target': target (if specified)
            - 'option': option (if provided)

    Example:
        >>> cmd = pan_tilt_step([5.0, -2.0], speed=30)
        >>> # {'topic': 'pan_tilt_step', 'value': [5.0, -2.0], 'option': {'speed': 30}}
    """
    command = {
        'topic': 'pan_tilt_step',
        'value': pan_tilt_angle_step
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def pan_tilt_zoom(pan_tilt_zoom_list: list[float], target: str = "all", **option) -> dict:
    """
    Set absolute pan, tilt, and zoom.

    Parameters:
        pan_tilt_zoom_list (list of float): [pan, tilt, zoom] where
            - pan (float): Pan angle in degrees.
            - tilt (float): Tilt angle in degrees.
            - zoom (float): Zoom ratio (optical zoom or focal length).
        target (str): Robot ID or "all" to broadcast. Defaults to "all".
        **option: Additional parameters (see pan_tilt documentation).

    Returns:
        dict: Command object with keys:
            - 'topic': 'pan_tilt_zoom'
            - 'value': pan_tilt_zoom_list
            - 'target': target (if specified)
            - 'option': option (if provided)

    Example:
        >>> cmd = pan_tilt_zoom([10.0, 0.0, 1.5], acceleration=60)
        >>> # {'topic': 'pan_tilt_zoom', 'value': [10.0, 0.0, 1.5], 'target': 'all', 'option': {'acceleration': 60}}
    """
    command = {
        'topic': 'pan_tilt_zoom',
        'value': pan_tilt_zoom_list
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def zoom(zoom_factor: float, target: str = "all", **option) -> dict:
    """
    Adjust zoom only while keeping pan and tilt unchanged.

    Parameters:
        zoom_factor (float): Zoom ratio (e.g., 1.0 = no zoom).
        target (str): Robot ID or "all" to broadcast. Defaults to "all".
        **option: Additional parameters (see pan_tilt documentation).

    Returns:
        dict: Command object with keys:
            - 'topic': 'zoom'
            - 'value': zoom_factor
            - 'target': target (if specified)
            - 'option': option (if provided)

    Example:
        >>> cmd = zoom(2.0, target="spot_cam_lens", speed=5)
        >>> # {'topic': 'zoom', 'value': 2.0, 'target': 'spot_cam_lens', 'option': {'speed': 5}}
    """
    command = {
        'topic': 'zoom',
        'value': zoom_factor
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command