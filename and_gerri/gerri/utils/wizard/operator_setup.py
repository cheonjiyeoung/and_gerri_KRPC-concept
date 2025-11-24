import os
import re
import json
import ast
import glob
import zipfile
from pathlib import Path


ROBOT_TYPE_MAP = {
    1: "Mobile",
    2: "Manipulator",
    3: "Mobile + Manipulator",
    4: "Another (Unsupport)"
}

# -------------------------------------------------------
# Query functions
# -------------------------------------------------------
def query_robot_model():
    while True:
        v = input("1. What is your Robot Model ?\n >>> ").strip()
        if v:
            return v
        print("Invalid Robot Model.\n")

def query_robot_id():
    while True:
        v = input("2. What is your Robot ID ?\n >>> ").strip()
        if v:
            return v
        print("Invalid Robot ID.\n")

def query_robot_type():
    msg = f"""
3. What is your Robot Type ?
================================
1. {ROBOT_TYPE_MAP[1]}
2. {ROBOT_TYPE_MAP[2]}
3. {ROBOT_TYPE_MAP[3]}
4. {ROBOT_TYPE_MAP[4]}
================================
>>> """
    while True:
        v = input(msg)
        try:
            v = int(v)
            if v in ROBOT_TYPE_MAP:
                return v
        except:
            pass
        print("Invalid Robot Type.\n")

def query_rubberneck_id():
    while True:
        v = input("4. What is your RubberNeck ID ?\n >>> ").strip()
        if v:
            return v
        print("Invalid RubberNeck ID.\n")

def query_rubberneck_pwd():
    while True:
        v = input("5. What is your RubberNeck Password ?\n >>> ").strip()
        if v:
            return v
        print("Invalid Password.\n")

def query_rubberneck_api_key():
    while True:
        v = input("6. What is your RubberNeck API Key ?\n >>> ").strip()
        if v:
            return v
        print("Invalid API Key.\n")

# -------------------------------------------------------
# Utility
# -------------------------------------------------------
def safe_dir_name(name: str) -> str:
    return re.sub(r'[^A-Za-z0-9_\-]', '_', name)

def to_class_name(name: str) -> str:
    parts = re.split(r'[^a-zA-Z0-9]+', name)
    return ''.join(p.capitalize() for p in parts if p)

def to_variable_name(class_name: str) -> str:
    s1 = re.sub('([A-Z]+)([A-Z][a-z])', r'\1_\2', class_name)
    s2 = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1)
    return s2.lower()

# -------------------------------------------------------
# Extract callable functions from sub_controller
# -------------------------------------------------------
def extract_functions(file_path: str):
    text = Path(file_path).read_text(encoding="utf-8")
    tree = ast.parse(text)
    funcs = {}

    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            name = node.name
            if name.startswith("_"):
                continue

            params = {}
            for a in node.args.args:
                if a.arg != "self":
                    params[a.arg] = {}

            # default 값 파싱
            if node.args.defaults:
                offset = len(params) - len(node.args.defaults)
                keys = list(params.keys())
                for i, default in enumerate(node.args.defaults):
                    arg_name = keys[i + offset]
                    try:
                        value = ast.literal_eval(default)
                    except Exception:
                        value = None
                    params[arg_name]["default"] = value

            funcs[name] = {"params": params}

    return funcs

# -------------------------------------------------------
# Generate operator config
# -------------------------------------------------------
def make_config_file(config, path):
    template_path = "./wizard/templates/operator_config.py"
    with open(template_path, "r", encoding="utf-8") as f:
        text = f.read()

    text = text.replace("'__ROBOT_ID__'", "'" + config["robot_id"] + "'")
    text = text.replace("'__ROBOT_MODEL__'", "'" + config["robot_model"] + "'")
    text = text.replace("'__ROBOT_CATEGORY__'", "'" + config["robot_type"] + "'")
    text = text.replace("'__RUBBERNECK_ID__'", "'" + config["rubberneck_id"] + "'")
    text = text.replace("'__RUBBERNECK_PASSWARD__'", "'" + config["rubberneck_pwd"] + "'")

    with open(path, "w", encoding="utf-8") as f:
        f.write(text)

# -------------------------------------------------------
# Generate Commander file
# -------------------------------------------------------
def make_commander_file(class_name, funcs, path):
    template_path = "./wizard/templates/operator_commander.py"
    with open(template_path, "r", encoding="utf-8") as f:
        base = f.read().replace("__COMMANDER__", class_name + "Commander")

    out = [base]
    out.append("\n#################################################################\n")
    out.append("# Auto-generated methods by sub_controller.py\n")
    out.append("#################################################################\n\n")

    for fname, info in funcs.items():
        params = info["params"]

        sig = f"    def {fname}(self"
        for p, attr in params.items():
            if "default" in attr:
                sig += f", {p}={repr(attr['default'])}"
            else:
                sig += f", {p}"
        sig += "):\n"

        body = []
        body.append(f'        topic = "{fname}"\n')
        body.append("        value = {}\n")
        for p in params:
            body.append(f'        value["{p}"] = {p}\n')
        body.append("        self.send_message(topic=topic, value=value)\n\n")

        out.append(sig)
        out.extend(body)

    # ← 리스트를 문자열로 합쳐서 write 해야 한다
    with open(path, "w", encoding="utf-8") as f:
        f.write("".join(out))

# -------------------------------------------------------
# Generate operator launch file
# -------------------------------------------------------
def make_operator_launch_file(robot_model, operator_dir, path):
    cls = to_class_name(robot_model)
    var = to_variable_name(cls)
    config_path = f"{robot_model}.{operator_dir}.{robot_model}_config"
    cmd_path = f"{robot_model}.{operator_dir}.{robot_model}_commander"

    template_path = "./wizard/templates/gerri_operator.py"
    with open(template_path, "r", encoding="utf-8") as f:
        text = f.read()

    text = (
        text.replace("__config_path__", config_path)
            .replace("__commander_path__", cmd_path)
            .replace("__commander_class__", cls+"Commander")
            .replace("__commander_var__", var)
    )

    with open(path, "w", encoding="utf-8") as f:
        f.write(text)

# -------------------------------------------------------
# Setup Wizard
# -------------------------------------------------------
def setup_wizard():
    model = query_robot_model()
    if not os.path.exists(model):
        print("ERROR : You don't setup gerri_robot yet.")
        return

    return {
        "robot_model": model,
        "robot_id": query_robot_id(),
        "robot_type": ROBOT_TYPE_MAP[query_robot_type()],
        "rubberneck_id": query_rubberneck_id(),
        "rubberneck_pwd": query_rubberneck_pwd(),
    }

# -------------------------------------------------------
# Main (with ZIP creation)
# -------------------------------------------------------
def main(config):
    import datetime

    robot_model = config["robot_model"]
    safe = safe_dir_name(robot_model)
    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

    operator_dir = os.path.join(safe, f"operator_{timestamp}")
    os.makedirs(operator_dir)

    # __init__.py
    Path(os.path.join(operator_dir, "__init__.py")).write_text("", encoding="utf-8")

    # operator config
    cfg_path = os.path.join(operator_dir, f"{robot_model}_config.py")
    make_config_file(config, cfg_path)

    # commander 생성
    robot_dir = os.path.join(safe, "robot")
    matches = glob.glob(os.path.join(robot_dir, "*_sub_controller.py"))
    if not matches:
        print("ERROR : Can not find sub_controller file")
        return

    sc_path = matches[0]
    funcs = extract_functions(sc_path)
    commander_path = os.path.join(operator_dir, f"{robot_model}_commander.py")
    make_commander_file(to_class_name(robot_model), funcs, commander_path)

    # operator entry 생성
    launch_path = f"{robot_model}_operator.py"
    make_operator_launch_file(robot_model, f"operator_{timestamp}", launch_path)

    # # ZIP 생성
    # zip_name = f"{robot_model}_operator_bundle_{timestamp}.zip"
    # with zipfile.ZipFile(zip_name, 'w', zipfile.ZIP_DEFLATED) as z:
    #     for root, _, files in os.walk(operator_dir):
    #         for f in files:
    #             full = os.path.join(root, f)
    #             arc = os.path.relpath(full, safe)
    #             z.write(full, arcname=arc)
    #     z.write(launch_path, arcname=launch_path)

    # print(f"Operator zip package created: {zip_name}")


if __name__ == "__main__":
    cfg = setup_wizard()
    if cfg:
        main(cfg)
