Import("env")

import os
import subprocess

build_dir = env.subst("$BUILD_DIR")
project_dir = env.subst("$PROJECT_DIR")
idf_path = env.PioPlatform().get_package_dir("framework-espidf")
script = os.path.join(idf_path, "tools", "cmake", "scripts", "data_file_embed_asm.cmake")

os.makedirs(build_dir, exist_ok=True)

embeds = (
    ("components/hsg_api/ESP32-POE.html", "ESP32-POE.html.S", "TEXT"),
    ("components/hsg_api/favicon.ico", "favicon.ico.S", "BINARY"),
    ("components/hsg_api/control.html", "control.html.S", "TEXT"),
)

for src_rel, out_name, file_type in embeds:
    src = os.path.join(project_dir, src_rel.replace("/", os.sep))
    out = os.path.join(build_dir, out_name)
    subprocess.run(
        [
            "cmake",
            f"-DDATA_FILE={src}",
            f"-DSOURCE_FILE={out}",
            f"-DFILE_TYPE={file_type}",
            "-P",
            script,
        ],
        check=True,
    )
