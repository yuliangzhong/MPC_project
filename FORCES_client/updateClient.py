# (c) Embotech AG, Zurich, Switzerland, 2013-2021

import sys
import forcespro  # required to adjust PYTHONPATH
import forcespro.updateClient

if __name__ == "__main__":
    forcespro.updateClient.main(sys.argv)
    sys.exit(0)
