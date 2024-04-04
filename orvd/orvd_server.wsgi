import sys
import logging
sys.path.insert(0,'/var/www/orvd')
logging.basicConfig(stream=sys.stderr)

from orvd_server import app as application