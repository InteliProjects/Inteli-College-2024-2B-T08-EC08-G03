---
title: Backend
sidebar_position: 4 
slug: "/sprint-3/backend"
---

## Introdução
Este backend foi projetado para:
- Integrar-se ao GPT para processar comandos e gerar respostas.
- Movimentar um robô para posições específicas com base em comandos.
- Receber mensagens via WhatsApp através do Twilio.
Esta sessão da documentação é destinada aos serviços de integração com o whatsapp e o LLM

## Endpoints

### 1. Webhook WhatsApp
**Método:** `POST`  
**URL:** `/twilio/webhook`  
**Descrição:** Recebe mensagens do WhatsApp e processa comandos, chamando os servicos de LLM.  
**Parâmetros do Corpo (JSON):**
```json
{
  "Body": "Mova o robô para posição 3",
  "From": "+5511999999999",
  "To": "+5511988888888"
}
```
**Resposta:**
```json
{
  "response": "Robô movido para posição 3."
}
```

### 2. Geração de Resposta GPT

**Método:** `POST`  
**URL:** `/gpt/generate`  

**Descrição:** Envia um prompt ao GPT e retorna uma resposta ou ação sugerida.  

**Parâmetros do Corpo (JSON):**
```json
{
  "prompt": "Mova o robô para posição 3"
}
```
**Resposta:**
```json
{
  "response": "Robô movido para posição 3."
}
```
## Serviços e Actions

### 1. GPT Service

**Descrição:**  
O serviço GPT é responsável por se conectar ao modelo OpenAI para processar prompts enviados pelos usuários e gerar respostas apropriadas. Ele também pode sugerir ações no formato JSON, como chamar uma função específica (por exemplo, mover um robô).

**Lógica de Funcionamento:**  
- O serviço envia mensagens (incluindo um histórico) ao GPT para análise e resposta.
- Caso o GPT identifique uma ação a ser executada, ele retorna no seguinte formato:
  ```json
  {
    "response": "Mensagem para o usuário explicando a ação.",
    "action": {
      "name": "nomeDaAcao",
      "parameters": {
        "chave": "valor"
      }
    }
  }
```
