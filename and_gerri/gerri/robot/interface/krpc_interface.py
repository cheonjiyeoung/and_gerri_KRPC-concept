from pubsub import pub
import inspect
import types

def _validate_parameters(method, value):
    """
    Validate that all required parameters of a given method are provided in the input value.

    This function inspects the signature of a method and checks whether every
    required (non-default) parameter is present in the given `value` dictionary.
    Optional parameters with default values are ignored during validation.

    Args:
        method (callable): The method whose parameters are to be validated.
        value (dict): A dictionary containing parameter names and corresponding
            argument values intended for the method.

    Returns:
        bool: 
            - True if all required parameters are present in `value`.  
            - False if one or more required parameters are missing.

    Example:
        >>> def move(x, y, speed=1.0): 
        ...     pass
        >>> _validate_parameters(move, {"x": 1, "y": 2})
        True
        >>> _validate_parameters(move, {"x": 1})
        False
    """
    params = inspect.signature(method).parameters
    for param, attr in params.items():
        if attr.default is inspect.Parameter.empty and param not in value:
            return False
    if not set(value.keys()).issubset(params.keys()):
        return False
    return True

class KrpcInterface:
    """
    Handling messages from KETIRTC command_channel
    This Class inclue Robot interfaces(SDK, robot_controll code, robot_sub_controll ...
    """
    def __init__(self,robot_info,robot_interface):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        self.robot_interface = robot_interface
        pub.subscribe(self.receive_message,"receive_message")

    def connect(self):
        try:
            self.robot_interface.connect()
        except:
            pass

    def disconnect(self):
        try:
            self.robot_interface.disconnect()
        except:
            pass

    def receive_message(self, message):
        """
        Handle an incoming message and dynamically invoke a corresponding method
        of the robot interface.

        This method is typically used in multi-robot control systems.  
        When a message is received, it checks whether the given topic corresponds
        to a method in the robot interface. If so, the method is invoked with
        the provided parameters. Execution occurs only if the message target
        matches the current robot's ID.

        Args:
            message (dict): The received message containing the following keys:
                - topic (str): Name of the method in the robot interface to invoke.
                - value (dict): Parameters to pass to the method.
                - target (str): Target robot ID for multi-robot execution.

        Raises:
            ValueError: If the method requires parameters that are missing in `value`.
            AttributeError: If the specified topic does not exist in the robot interface.
            TypeError: If the parameters do not match the method signature.

        Example:
            >>> msg = {
            ...     "topic": "move_to",
            ...     "value": {"x": 1.2, "y": 3.4},
            ...     "target": "robot_01"
            ... }
            >>> robot.receive_message(msg)
            # Calls self.robot_interface.move_to(x=1.2, y=3.4)
        """

        topic = message.get("topic")    # If the topic is a method of the robot interface, invoke that method.
        if topic == "/joy":
            return
        value = message.get("value")    # Parameters to be passed when invoking the robot interface method.
        
        if type(self.robot_interface) == dict:
            target = message.get("target")  # In multi-robot control, execution is performed based on the robot IDs included in the target.
            if not target in self.robot_interface.keys() or target is None:
                print("""
                    No robot matching target found.
                    If you want to control multiple robots, pass the robot interface as a dictionary ( {<robot_id> : <instance or module>} )
                    and enter the exact robot ID in the target field.
                    """)
                return
            else:
                target_robot_interface = self.robot_interface.get(target)
        else:
            target_robot_interface = self.robot_interface

        if inspect.isclass(target_robot_interface):
            print("robot_interface is a class, not an instance. Did you forget to instantiate it?")
            return
        
        method = getattr(target_robot_interface, topic, None)

        # Check whether the robot interface has the method.
        if not callable(method):
            print(f"Robot interface has no callable method or function '{topic}'")
            return
        # Get the method parameters and parse the received message.
        params = inspect.signature(method).parameters

        try:
            if len(params) == 0:
                method()
            elif _validate_parameters(method, value):
                method(**value)
            else:
                print(f"Missing required parameters for '{topic}'.")
                print(f"Expected: {list(params.keys())}, Provided: {list(value.keys())}")
        except Exception as e:
            print(f"Error calling '{topic}': {e}")

                    
    def send_message(self, message):
        """Send message to Operator"""
        pub.sendMessage('send_message', message=message)