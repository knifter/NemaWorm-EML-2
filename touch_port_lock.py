Import("env")

import datetime

STAMP_FILE = "release_port.lock"

def touch_stamp(source, target, env):
    with open(STAMP_FILE, "w") as f:
        f.write(datetime.datetime.now().isoformat())
    print("Touched %s" % STAMP_FILE)

# post build: after the firmware binary is produced
env.AddPostAction("buildprog", touch_stamp)

# pre upload: just before flashing
env.AddPreAction("upload", touch_stamp)