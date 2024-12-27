from flask import Blueprint, jsonify, request
from app.services import (
    move_robot,
    get_robot_position,
)

main_blueprint = Blueprint("main", __name__)


@main_blueprint.route("/move", methods=["POST"])
async def move_robot_route():
    """
    Move o robô para uma lista de posições específicas.

    Payload JSON:
    [
        {"x": 1.0, "y": 1.0, "orientation": 0.0},
        {"x": 2.0, "y": -1.0, "orientation": 1.57}
    ]

    Returns:
        - 202 Accepted: Movimento iniciado com sucesso.
        - 400 Bad Request: Payload inválido.
    """
    data = request.get_json()

    if not isinstance(data, list) or not all(
        isinstance(pos, dict) and "x" in pos and "y" in pos and "orientation" in pos
        for pos in data
    ):
        return jsonify({"error": "Payload deve ser uma lista de objetos com x, y e orientation"}), 400

    try:
        positions = [
            {
                "x": float(pos["x"]),
                "y": float(pos["y"]),
                "orientation": float(pos["orientation"]),
            }
            for pos in data
        ]
    except (ValueError, TypeError):
        return jsonify({"error": "Os valores de x, y e orientation devem ser números válidos"}), 400

    try:
        move_robot(positions)
        return (
            jsonify(
                {"message": f"Movimento feito para {len(positions)} posições."}
            ),
            202,
        )
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@main_blueprint.route("/position", methods=["GET"])
async def get_position():
    """
    Retorna a posição atual do robô
    """

    position = await get_robot_position()
    print(position)

    return jsonify({"position": position}), 200
