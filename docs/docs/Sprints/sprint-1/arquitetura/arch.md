---
title: Arquitetura da solução
sidebar_position: 1
slug: "/arch"
---

### Arquitetura do Sistema do Robô Autônomo

#### 1. **Interface de Usuário e Frontend**
   - **Tecnologias**: Next.js, TypeScript, ShadCN UI, Tailwind CSS, tRPC
   - **Descrição**: A interface de usuário em Next.js permite a interação dos pacientes e técnicos de manutenção com o robô, servindo tanto para o fallback (caso comandos de voz não funcionem) quanto para monitoramento remoto. Usando TypeScript e ShadCN UI, o sistema oferece um design moderno e uma interface responsiva, adaptada para tablets ou celulares.

#### 2. **Backend**
   - **Tecnologia**: Next.js
   - **Descrição**: O backend, também desenvolvido com Next.js, gerencia as comunicações entre os diversos módulos do sistema e processa requisições do frontend. Ele serve como ponto central de comunicação com o robô, acessa APIs, processa dados e mantém persistência para logs e estados do sistema.

#### 3. **Processamento de Voz e Comandos**
   - **Tecnologia**: IBM Watson (ou alternativa de LLM)
   - **Descrição**: O IBM Watson (ou LLM alternativo) processa os comandos de voz e converte as instruções em ações que o sistema do robô executa. Este módulo é responsável por interpretar os comandos em linguagem natural e adaptá-los para navegação e interação amigável.

#### 4. **Navegação e Controle do Robô**
   - **Tecnologia**: ROS (Robot Operating System), SLAM (C++)
   - **Descrição**: Este módulo lida com o movimento do robô no ambiente hospitalar. O SLAM permite que o robô construa e utilize um mapa do ambiente para navegação autônoma e evita colisões. Através do ROS, o sistema integra diferentes sensores e motores, e se comunica com o backend para ajustar rotas e monitorar status.

#### 5. **Comunicação e Integração Geral**
   - **Protocolos**: WebSocket para comunicação em tempo real, REST API para integração com sistemas externos e hospitalares
   - **Descrição**: A comunicação entre frontend, backend e sistema de controle do robô é realizada com WebSocket para atualizações em tempo real e REST APIs para integração e extensibilidade. 

### Diagrama de Blocos

![diagrama](/img/sprint-1/arch.png)

### Explicação do Diagrama

1. **Frontend**: A interface do usuário envia e recebe dados do backend utilizando o tRPC, facilitando a interação dos usuários com o sistema.
2. **Backend**: O backend em Next.js centraliza as comunicações, realizando o processamento de dados, a integração com APIs e a comunicação com o controle de navegação.
3. **Processamento de Voz**: O LLM processa comandos de voz e os envia ao backend, possibilitando a conversão de linguagem natural em comandos compreensíveis pelo sistema.
4. **Controle de Navegação**: ROS e SLAM trabalham juntos para que o robô navegue de forma autônoma, com ROS gerenciando a comunicação entre os diferentes sensores e atuadores e o SLAM realizando o mapeamento e a localização.
5. **Integração e Comunicação**: A arquitetura usa WebSocket para atualizações em tempo real e REST API para conectar o robô a sistemas hospitalares.

Essa arquitetura é modular, permitindo upgrades e manutenções em cada componente sem afetar o sistema como um todo.
