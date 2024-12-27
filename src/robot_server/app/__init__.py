from flask import Flask
from app.routes import main_blueprint
from app.errors import register_error_handlers
from app.config import Config

def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)

    app.register_blueprint(main_blueprint)
    register_error_handlers(app)

    return app
