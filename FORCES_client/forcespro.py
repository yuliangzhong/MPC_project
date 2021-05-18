"""
Python Interface to FORCESPRO Code Genertor

https://forces.embotech.com/Documentation/

This module loads the Python client to the FORCESPRO solver generator.

Note that the Client package is distributed as several ZIP
files, one for each supported Python version (e.g. `forcespro36.zip`). There is
no need to unzip these ZIP-folders; simply put them together with this loader
script in your Python path or project directory.  Executing `import forcespro`
first loads the file you are looking at; in this file, we add the ZIP-folder
with the correct version to the Python path, then load the actual `forcespro`
package from the ZIP-folder.

For better IDE support, you may still want to unzip the ZIP-folder for your
correct Python version, though.

(c) Embotech AG, Zurich, Switzerland, 2013-2021
"""

import sys
import os.path
try:
    reload = reload  # Python 2.7
except NameError:
    import importlib
    reload = importlib.reload

forces_dir = os.path.dirname(os.path.realpath(__file__))

# Version string as used in our zip files
version = "{}{}".format(sys.version_info[0], sys.version_info[1])

# After building using the `package_python_client.py` script, we have a ZIP
# folder `modules.zip` containing the required modules; add this to path.
submodule_dir = os.path.join(forces_dir, "forcespro{}.zip".format(version))
sys.path.insert(0, submodule_dir)

# This file has the same name as the package we are trying to load. After adding
# the ZIP-file to the path, `forcespro` will now refer to the package, not this
# file any more. The following trick loads the package `forcespro` in the ZIP in
# the top-level.
import forcespro
reload(forcespro)
forcespro._set_forces_dir(forces_dir)
