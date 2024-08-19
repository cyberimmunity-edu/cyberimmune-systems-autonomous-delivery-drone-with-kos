import sys
import logging
sys.path.insert(0,'/var/www/orvd')
logging.basicConfig(stream=sys.stderr)

from orvd_server import create_app, clean_app_db

application = create_app()
clean_app_db(application)
