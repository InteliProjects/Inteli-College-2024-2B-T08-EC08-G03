---
title: Backend
slug: "/backend"
---

## Introdução
Este backend foi implementado para:
- Integrar-se ao GPT para interpretar comandos complexos com múltiplos destinos
- Movimentar um robô entre locais pré-cadastrados para executar tarefas de coleta e entrega
- Receber mensagens via WhatsApp através do Twilio
Esta sessão da documentação é destinada aos serviços de integração com o WhatsApp e o LLM

## Endpoints

### 1. Webhook WhatsApp
**Método:** `POST`  
**URL:** `/twilio/webhook`  
**Descrição:** Recebe mensagens do WhatsApp, processa comandos complexos de múltiplas etapas e aciona os serviços de LLM.  
**Parâmetros do Corpo (JSON):**
```json
{
  "Body": "Pegue uma almofada e leve para a sala 2",
  "From": "+5511999999999",
  "To": "+5511988888888"
}
```
**Resposta:**
```json
{
  "response": "Iniciando rota: 1) Indo ao armazém buscar almofada 2) Entregando na sala 2"
}
```

### 2. Geração de Resposta GPT
**Método:** `POST`  
**URL:** `/gpt/generate`  
**Descrição:** Analisa comandos complexos, identifica objetos e locais, e gera uma rota com múltiplas paradas.  
**Parâmetros do Corpo (JSON):**
```json
{
  "prompt": "Pegue uma almofada e leve para a sala 2"
}
```
**Resposta:**
```json
{
  "response": "Rota planejada para coleta e entrega",
  "route": {
    "steps": [
      {
        "type": "pickup",
        "location": {
          "name": "armazém",
          "coordinates": {
            "x": 100,
            "y": 200
          }
        },
        "item": "almofada"
      },
      {
        "type": "delivery",
        "location": {
          "name": "sala 2",
          "coordinates": {
            "x": 300,
            "y": 150
          }
        }
      }
    ]
  }
}
```

## Serviços e Actions

### 1. GPT Service
**Descrição:**  
O serviço GPT foi aprimorado para processar comandos complexos que envolvem múltiplas ações e locais. Ele identifica itens a serem coletados, locais de origem e destino, gerando uma rota otimizada.

**Lógica de Funcionamento:**  
- O serviço recebe a mensagem do usuário via WhatsApp
- O GPT analisa a mensagem e identifica:
  - Ação principal (pegar/levar)
  - Item a ser transportado
  - Local de coleta (baseado no tipo do item)
  - Local de entrega
- O sistema consulta o banco de dados para cada localização
- Uma rota completa é gerada com todos os pontos de parada
- O sistema retorna no seguinte formato:
  ```json
  {
    "response": "Mensagem descrevendo a rota completa",
    "action": {
      "name": "executeRoute",
      "parameters": {
        "route": [
          {
            "type": "pickup",
            "location": "armazém",
            "coordinates": {
              "x": 100,
              "y": 200
            },
            "item": "almofada"
          },
          {
            "type": "delivery",
            "location": "sala 2",
            "coordinates": {
              "x": 300,
              "y": 150
            }
          }
        ]
      }
    }
  }
  ```

### 2. Location Service
**Descrição:**  
Serviço responsável por gerenciar o mapeamento entre nomes de locais e suas coordenadas no sistema, agora incluindo informações sobre itens disponíveis em cada local.

**Estrutura do Banco de Dados:**
```sql
CREATE TABLE locations (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100) NOT NULL,
    x_coord INTEGER NOT NULL,
    y_coord INTEGER NOT NULL,
    description TEXT,
    location_type VARCHAR(50)
);

CREATE TABLE items (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100) NOT NULL,
    default_pickup_location_id INTEGER REFERENCES locations(id),
    description TEXT
);
```

**Exemplo de Consulta:**
```sql
-- Encontrar local de coleta para um item
SELECT l.name, l.x_coord, l.y_coord 
FROM locations l
JOIN items i ON l.id = i.default_pickup_location_id
WHERE i.name = 'almofada';

-- Encontrar coordenadas do local de entrega
SELECT x_coord, y_coord 
FROM locations 
WHERE name = 'sala 2';
```

### 3. Route Service
**Descrição:**  
Novo serviço implementado para gerenciar rotas com múltiplos pontos de parada.

**Funcionalidades:**
- Otimização de rota para minimizar distância total
- Validação de disponibilidade de itens no local de coleta
- Monitoramento de progresso da rota
- Notificação de conclusão de cada etapa

# Backend do robô

Além dessas alterações, adicionamos o suporte a listas de posições no backend do robô, sendo possível enviar uma lista de posições para o robô e ele irá percorrer todas elas em sequência.

**Método:** `POST`

**URL:** `/robot/move`

**Descrição:** Envia uma lista de posições para o robô percorrer

**Parâmetros do Corpo (JSON):**
```json
{
  "positions": [
    {
      "x": 100,
      "y": 200,
      "orientation": 90
    },
    {
      "x": 300,
      "y": 150,
      "orientation": 0
    }
  ]
}
```
