import os
import sys
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
project_dir = env['PROJECT_DIR']

# Path to your hook source file
src_hook = os.path.join(project_dir, "scripts", "pre-commit.hook")
# Destination where git expects the hook
dest_hook = os.path.join(project_dir, ".git", "hooks", "pre-commit")

# Ensure the .git/hooks directory exists
os.makedirs(os.path.dirname(dest_hook), exist_ok=True)

# Remove any existing file or symlink at the destination
if os.path.lexists(dest_hook):
    os.remove(dest_hook)

try:
    os.symlink(src_hook, dest_hook)
    print("Created symlink: {} -> {}".format(dest_hook, src_hook))
except Exception as e:
    print("Error creating symlink:", e)
    sys.exit(1)
