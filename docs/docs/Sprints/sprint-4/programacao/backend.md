
---
title: Backend
slug: "/backend"
---

## Introdução
Este backend foi implementado para:
- Integrar-se ao GPT para processar comandos e gerar respostas inteligentes
- Movimentar um robô para locais pré-cadastrados com base em comandos em linguagem natural
- Receber mensagens via WhatsApp através do Twilio
Esta sessão da documentação é destinada aos serviços de integração com o WhatsApp e o LLM

## Endpoints

### 1. Webhook WhatsApp
**Método:** `POST`  
**URL:** `/twilio/webhook`  
**Descrição:** Recebe mensagens do WhatsApp, processa comandos de localização e aciona os serviços de LLM.  
**Parâmetros do Corpo (JSON):**
```json
{
  "Body": "Mova o robô para o laboratório",
  "From": "+5511999999999",
  "To": "+5511988888888"
}
```
**Resposta:**
```json
{
  "response": "Robô está se movendo para o laboratório nas coordenadas X:120, Y:350."
}
```

### 2. Geração de Resposta GPT
**Método:** `POST`  
**URL:** `/gpt/generate`  
**Descrição:** Processa o comando de localização via GPT e retorna as coordenadas correspondentes do local solicitado.  
**Parâmetros do Corpo (JSON):**
```json
{
  "prompt": "Mova o robô para o laboratório"
}
```
**Resposta:**
```json
{
  "response": "Movendo robô para o laboratório",
  "location": {
    "name": "laboratório",
    "coordinates": {
      "x": 120,
      "y": 350
    }
  }
}
```

## Serviços e Actions

### 1. GPT Service
**Descrição:**  
O serviço GPT foi implementado para processar comandos em linguagem natural, identificar locais mencionados e coordenar o movimento do robô através da integração com o banco de dados de localizações.

**Lógica de Funcionamento:**  
- O serviço recebe a mensagem do usuário via WhatsApp
- O GPT identifica o local mencionado na mensagem
- O sistema consulta o banco de dados para obter as coordenadas do local
- Uma resposta é gerada incluindo os detalhes da movimentação
- O sistema retorna no seguinte formato:
  ```json
  {
    "response": "Mensagem confirmando movimento para o local solicitado",
    "action": {
      "name": "moveToLocation",
      "parameters": {
        "locationName": "laboratório",
        "coordinates": {
          "x": 120,
          "y": 350
        }
      }
    }
  }
  ```

### 2. Location Service
**Descrição:**  
Serviço responsável por gerenciar o mapeamento entre nomes de locais e suas coordenadas no sistema.

**Estrutura do Banco de Dados:**
```sql
CREATE TABLE locations (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100) NOT NULL,
    x_coord INTEGER NOT NULL,
    y_coord INTEGER NOT NULL,
    description TEXT
);
```

**Exemplo de Consulta:**
```sql
SELECT x_coord, y_coord 
FROM locations 
WHERE name = 'laboratório';
```
