import sys
from pathlib import Path
__PKG_PATH = Path(__file__).parent.parent
if not __PKG_PATH not in sys.path:
    sys.path.append(__PKG_PATH)