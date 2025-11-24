from wizard.templates import robot_config
import os
import re
import json

ROBOT_TYPE_MAP = {
    1: "Mobile",
    2: "Manipulator",
    3: "Mobile + Manipulator",
    4: "Another (Unsupport)"
}

ROBOT_TYPE_TEMPLATE_MAP = {
    1: "./wizard/templates/mobile_sub_controller.py",
    2: "./wizard/templates/manipulator_sub_controller.py",
    3: "./wizard/templates/mobile_manipulator_sub_controller.py",
    4: "./wizard/templates/base_sub_controller.py"
}

context_query_robot_model = "1. What is your Robot Model ?\n >>> "
context_query_robot_id = "2. What is your Robot ID ?\n >>> "


context_query_robot_type = f"""
3. What is your Robot Type ?
================================
1. {ROBOT_TYPE_MAP[1]}
2. {ROBOT_TYPE_MAP[2]}
3. {ROBOT_TYPE_MAP[3]}
4. {ROBOT_TYPE_MAP[4]}
================================
>>> """

context_query_rubberneck_api_key = "4. What is your RubberNeck API Key ?\n >>> "

# -------------------------------------------------------
# Query functions
# -------------------------------------------------------

def query_robot_id():
    while True:
        robot_id = input(context_query_robot_id)
        if robot_id.strip() == "":
            print("Invalid Robot ID. Try Again...\n")
        else:
            return robot_id.strip()
        
def query_robot_model():
    while True:
        robot_model = input(context_query_robot_model)
        if robot_model.strip() == "":
            print("Invalid Robot Model. Try Again...\n")
        else:
            return robot_model.strip()
        
def query_robot_type():
    while True:
        robot_type = input(context_query_robot_type)
        try:
            robot_type = int(robot_type)
        except:
            print("Invalid input. Must be a number.\n")
            continue
        
        if robot_type not in ROBOT_TYPE_MAP:
            print("Invalid Robot Type. Try Again...\n")
        else:
            return robot_type

def query_float(prompt, allow_zero=False):
    while True:
        try:
            value = float(input(prompt))
            if not allow_zero and value <= 0:
                print("Value must be greater than zero.\n")
                continue
            return value
        except ValueError:
            print("Invalid number. Try again.\n")


def query_max_linear_velocity():
    """
    0 입력 시 기본값 1.0 적용.
    """
    prompt = (
        "3. What is the MAX linear velocity of your robot? (m/s)\n"
        ">>> (Enter 0 to use default = 1.0)\n >>> "
    )
    value = query_float(prompt, allow_zero=True)

    if value == 0:
        return 1.0
    return value


def query_min_linear_velocity(max_value):
    """
    0 입력 시 기본값 0.1 적용.
    """
    prompt = (
        "3. What is the MIN linear velocity of your robot? (m/s)\n"
        ">>> (Enter 0 to use default = 0.1)\n >>> "
    )

    while True:
        value = query_float(prompt, allow_zero=True)

        if value == 0:
            return 0.1

        if value >= max_value:
            print("MIN velocity must be smaller than MAX velocity.\n")
        else:
            return value


def query_max_angular_velocity():
    """
    0 입력 시 기본값 1.0 적용.
    """
    prompt = (
        "3. What is the MAX angular velocity of your robot? (rad/s)\n"
        ">>> (Enter 0 to use default = 1.0)\n >>> "
    )
    value = query_float(prompt, allow_zero=True)

    if value == 0:
        return 1.0
    return value


def query_min_angular_velocity(max_value):
    """
    0 입력 시 기본값 0.1 적용.
    """
    prompt = (
        "3. What is the MIN angular velocity of your robot? (rad/s)\n"
        ">>> (Enter 0 to use default = 0.1)\n >>> "
    )

    while True:
        value = query_float(prompt, allow_zero=True)

        if value == 0:
            return 0.1

        if value >= max_value:
            print("MIN velocity must be smaller than MAX velocity.\n")
        else:
            return value


def query_manipulator_joint_count():
    while True:
        try:
            value = int(input("3. How many joints does your manipulator have?\n >>> "))
            if value <= 0:
                print("Joint count must be greater than zero.\n")
                continue
            return value
        except ValueError:
            print("Invalid number. Try again.\n")

def query_rubberneck_api_key():
    while True:
        api_key = input(context_query_rubberneck_api_key)
        if not api_key.strip():
            print("Invalid API Key. Try Again...\n")
        else:
            return api_key.strip()

# -------------------------------------------------------
# Create controller instance based on robot type
# -------------------------------------------------------
def create_robot_sub_controller(robot_type: int):
    """
    robot_type:
        1 → Mobile
        2 → Manipulator
        3 → Mobile + Manipulator
        4 → Unsupported
    """
    if robot_type == 4:
        print("Warning: Unsupported robot type selected.")
    elif ROBOT_TYPE_MAP.get(robot_type) is None:
        print("ERROR: Invaild robot type selected.")
    else:
        return robot_type


    
def format_dict_block(name, value):
    # 1. JSON pretty-print → { 가 자동으로 줄바꿈 내려감
    text = json.dumps(value, indent=4)

    # 2. JSON → Python literal 변환
    text = text.replace("null", "None")
    text = text.replace("true", "True")
    text = text.replace("false", "False")

    # 3. 변수명 + 줄바꿈 + dict block
    return f"{name} = {text}\n\n"

def make_config_file(config, path):
    # Inject values
    robot_config.ROBOT_INFO["id"]       = config["robot_id"]
    robot_config.ROBOT_INFO["model"]    = config["robot_model"]
    robot_config.ROBOT_INFO["category"] = config["robot_type"]
    robot_config.ROBOT_INFO["api_key"]  = config["rubberneck_api_key"]

    with open(path, "w", encoding="utf-8") as f:

        # ------------------------------
        # ROBOT_INFO
        # ------------------------------
        f.write("### ROBOT\n")
        f.write("# Defines robot identity, type and classification\n")
        f.write(format_dict_block("ROBOT_INFO", robot_config.ROBOT_INFO))

        # ------------------------------
        # VIDEO_INFO
        # ------------------------------
        f.write("### VIDEO\n")
        f.write("# Camera settings: device index and resolution\n")
        f.write("# Top-level key in VIDEO_INFO is the channel name.\n")
        f.write("# If top-level key = 'front_camera', it appears as <robot_id>_front_cam in RubberNeck.\n")
        f.write("# If you want to use a custom video source, set your own class at ['source'].\n")
        f.write("# If you want to use our data-collection service, add more camera data (pose, dFOV).\n")
        f.write(format_dict_block("VIDEO_INFO", robot_config.VIDEO_INFO))

        # ------------------------------
        # AUDIO_INFO
        # ------------------------------
        f.write("### AUDIO\n")
        f.write("# Audio I/O devices\n")
        f.write(format_dict_block("AUDIO_INFO", robot_config.AUDIO_INFO))

        # ------------------------------
        # Custom constants
        # ------------------------------
        f.write("# You can add any constant for your robot\n")

def to_class_name(name: str) -> str:
    """
    Convert snake_case (or any string with underscores) into PascalCase class name.
    ex) test_mobile -> TestMobile
    """
    parts = re.split(r'[^a-zA-Z0-9]+', name)  # underscore 또는 특수문자 기준 분리
    camel = ''.join(p.capitalize() for p in parts if p)
    return camel

def to_variable_name(class_name: str) -> str:
    """
    Convert PascalCase / CamelCase class name to pythonic snake_case variable name.
    ex)
        TestMobile -> test_mobile
        MyRobotController -> my_robot_controller
        HTMLParser -> html_parser
        HelloWorld123 -> hello_world123
    """
    # 1. ABCDef -> ABC_Def (연속 대문자 처리)
    s1 = re.sub('([A-Z]+)([A-Z][a-z])', r'\1_\2', class_name)
    # 2. aBc -> a_Bc (일반적인 대문자 앞에 언더스코어)
    s2 = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1)

    return s2.lower()

def make_sub_controller_file(config, path):
    robot_model = config["robot_model"]
    robot_type = config["sub_controller_template"]
    class_robot_model = to_class_name(robot_model)

    # 템플릿 모듈 선택
    # template_module = ROBOT_TYPE_TEMPLATE_MAP[robot_type]

    # 템플릿 파일 경로 가져오기
    template_path = ROBOT_TYPE_TEMPLATE_MAP[robot_type]
    # 템플릿 읽기
    with open(template_path, "r", encoding="utf-8") as f:
        template_code = f.read()

    # 클래스명 치환
    class_name = f"{class_robot_model}SubController"
    template_code = template_code.replace("__CLASS_NAME__", class_name)

    # -------- Mobile Parameters 삽입 --------
    if robot_type in [1, 3]:
        template_code = template_code.replace("'__MAX_LINEAR_VELOCITY__'", str(config["max_linear_velocity"]))
        template_code = template_code.replace("'__MIN_LINEAR_VELOCITY__'", str(config["min_linear_velocity"]))
        template_code = template_code.replace("'__MAX_ANGULAR_VELOCITY__'", str(config["max_angluar_velocity"]))
        template_code = template_code.replace("'__MIN_ANGULAR_VELOCITY__'", str(config["min_angluar_velocity"]))
    else:
        # 필요한 않는 경우 기본 치환(값 공백)
        template_code = template_code.replace("'__MAX_LINEAR_VELOCITY__'", "None")
        template_code = template_code.replace("'__MIN_LINEAR_VELOCITY__'", "None")
        template_code = template_code.replace("'__MAX_ANGULAR_VELOCITY__'", "None")
        template_code = template_code.replace("'__MIN_ANGULAR_VELOCITY__'", "None")

    # -------- Manipulator Parameters 삽입 --------
    if robot_type in [2, 3]:
        joint_map = {}
        for i in range(config["manipulator_joint_count"]):
            joint_map[i] = i
        template_code = template_code.replace("__JOINT_NAME_MAP__", str(joint_map))
    else:
        template_code = template_code.replace("__JOINT_NAME_MAP__", "None")

    # 파일로 저장
    with open(path, "w", encoding="utf-8") as f:
        f.write(template_code)

def make_robot_launch_file(config, path):
    robot_id = config["robot_id"]
    robot_model = config["robot_model"]
    class_robot_model = to_class_name(robot_model)

    # 템플릿 파일 경로 가져오기
    template_path = "./wizard/templates/gerri_robot.py"
    # 템플릿 읽기
    with open(template_path, "r", encoding="utf-8") as f:
        template_code = f.read()

    # config_path = robot_model + "_config"
    # sub_controller_path = robot_model + "_sub_controller"
    robot_config_path = robot_model + "." + "robot." + robot_model + "_config"
    robot_sub_controller_path = robot_model + "." + "robot." + robot_model + "_sub_controller"
    template_code = template_code.replace("__ROBOT_CONFIG_PATH__", robot_config_path)
    template_code = template_code.replace("__ROBOT_SUB_CONTROLLER_PATH__", robot_sub_controller_path)
    # template_code = template_code.replace("__ROBOT_CONFIG__", config_path)
    # template_code = template_code.replace("__ROBOT_SUB_CONTROLLER__", sub_controller_path)
    
    class_name = f"{class_robot_model}SubController"
    template_code = template_code.replace("__ROBOT_SUB_CONTROLLER_CLASS__", class_name)
    value = f"{class_name}(ROBOT_INFO['id'], ROBOT_INFO['model'], ROBOT_INFO['category'])"
    sub_controllers_str = f'{{"{robot_id}": {value}}}'
    template_code = template_code.replace("__SUB_CONTROLLERS__", sub_controllers_str)

    # 파일로 저장
    with open(path, "w", encoding="utf-8") as f:
        f.write(template_code)

# -------------------------------------------------------
# Final Wizard execution
# -------------------------------------------------------
def setup_wizard():
    robot_model = query_robot_model()
    robot_id = query_robot_id()
    robot_type = query_robot_type()
    max_linear_velocity = query_max_linear_velocity() if robot_type in [1,3] else None
    min_linear_velocity = query_min_linear_velocity(max_linear_velocity) if robot_type in [1,3] else None
    max_angluar_velocity = query_max_angular_velocity() if robot_type in [1,3] else None
    min_angluar_velocity = query_min_angular_velocity(max_angluar_velocity) if robot_type in [1,3] else None
    manipulator_joint_count = query_manipulator_joint_count() if robot_type in [2,3] else None
    rubberneck_api_key = query_rubberneck_api_key()

    sub_controller_template = create_robot_sub_controller(robot_type)

    config = {
        "robot_id": robot_id,
        "robot_model": robot_model,
        "robot_type": ROBOT_TYPE_MAP[robot_type],
        "rubberneck_api_key": rubberneck_api_key,
        "sub_controller_template": sub_controller_template,
        "max_linear_velocity":max_linear_velocity,
        "min_linear_velocity":min_linear_velocity,
        "max_angluar_velocity":max_angluar_velocity,
        "min_angluar_velocity":min_angluar_velocity,
        "manipulator_joint_count":manipulator_joint_count
    }

    print("\nRobot setup complete.")
    print("========================================\n")
    return config

def safe_dir_name(name: str) -> str:
    """
    Remove or replace unsafe characters from robot model name.
    Ex: "My Robot V1" → "My_Robot_V1"
    """
    return re.sub(r'[^A-Za-z0-9_\-]', '_', name)

def make_unique_dir(base_name: str, root_path="./"):
    """
    Create a unique directory:
    base_name → ./base_name
    If exists → ./base_name_1, ./base_name_2, ...
    Returns: the directory path created
    """
    base_name = safe_dir_name(base_name)
    dir_path = os.path.join(root_path, base_name)

    idx = 1
    while True:
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
            return dir_path
        else:
            dir_path = os.path.join(root_path, f"{base_name}_{idx}")
            idx += 1

def main(config):
    # 1. Make robot dir (dir name = robot model)
    robot_model = config.get("robot_model")
    target_dir = make_unique_dir(robot_model)
    robot_dir = target_dir + "/" + "robot"
    os.makedirs(robot_dir)
    with open(target_dir + "/__init__.py", "w") as file:
        pass
    with open(robot_dir + "/__init__.py", "w") as file:
        pass

    # 2. Make robot subcontroller template code
    sub_controller_file_path = robot_dir + "/" + robot_model + "_sub_controller.py"
    make_sub_controller_file(config,sub_controller_file_path)
    print(f"Created {robot_model}_subcontroller.py: {sub_controller_file_path}")

    # 3. Make robot config template code
    config_file_path = robot_dir + "/" + robot_model + "_config.py"
    make_config_file(config, config_file_path)
    print(f"Created {robot_model}_config.py: {config_file_path}")

    # 4. Make robot launch code
    launch_file_path = robot_model + "_robot.py"
    make_robot_launch_file(config, launch_file_path)
    print(f"Created {robot_model}_robot.py: {launch_file_path}")

    print(f"Setup Done!: You should modify {robot_dir}/{robot_model}_subcontroller.py and enjoy and-gerri!")
            

# If you want immediate run:
if __name__ == "__main__":
    setup_result = setup_wizard()
    main(setup_result)
