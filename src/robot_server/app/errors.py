from flask import jsonify

def register_error_handlers(app):
    @app.errorhandler(404)
    def not_found_error(error):
        return jsonify({"error": "Endpoint não encontrado"}), 404

    @app.errorhandler(500)
    def internal_error(error):
        return jsonify({"error": "Erro interno do servidor"}), 500
