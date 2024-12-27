---
title: Matriz de Riscos
sidebar_position: 2
slug: "/matriz-de-riscos"
---

# Introdução

&emsp;Este documento apresenta uma análise da Matriz de Riscos e Oportunidades relacionada ao desenvolvimento e implementação de um robô autônomo que utiliza IA para guiar pessoas dentro de um hospital. O robô se comunica com os pacientes e funcionários para entender seus destinos, confirma as direções por meio de uma interface visual e, em seguida, acompanha a pessoa até o local desejado. Após a conclusão da tarefa, o robô se desloca para a estação de carregamento mais próxima e fica à disposição para novas instruções. Esse projeto é realizado em parceria com a IBM.

**Observação: Todos os itens dispostos na matriz de risco serão descritos logo abaixo para melhor compreensão de onde pode-se encontrar os verdadeiros riscos/oportunidades e como mitigalos/potencializa-los**

<p align="center"><b> Figura 1 - Matriz de Riscos</b></p>
<div align="center" class="zoom-image">
  <img src={require('../../../../static/img/sprint-1/matrizDeRiscos.png').default} alt="matriz-de-riscos"/>
  <p><b>Fonte:</b> Elaborado por Grupo 3</p>
</div>

## Riscos

&emsp;Para melhor compreensão e eficiencia da analise da Matriz de Riscos foi desenvolvido um plano de prevenção para evitar que os riscos se concretizem. Os seguintes riscos foram identificados para o projeto de um robô autônomo que utiliza IA, cada um posicionado na matriz de acordo com sua probabilidade e impacto:

### Riscos de Alta Probabilidade e Alto Impacto

- O robô pode se perder ou colidir com obstáculos.
    - Explicação: Em um hospital, onde o ambiente está em constante movimento com pessoas e objetos, o robô precisa navegar com precisão. Falhas de navegação podem resultar em colisões, colocando em risco a segurança de pacientes e funcionários ou até danificando o robô.

    - Mitigação:
        - Sensores e algoritmos de detecção avançada: Implementar sensores LIDAR, câmeras e algoritmos SLAM (Simultaneous Localization and Mapping) para evitar colisões e melhorar a navegação.
        - Atualizações frequentes do software de navegação: Manter o software atualizado com melhorias em reconhecimento de obstáculos.
        - Mapeamento dinâmico em tempo real: Criar mapas atualizáveis em tempo real, garantindo que o robô se adapte a mudanças no ambiente hospitalar.

### Riscos de Probabilidade Moderada e Impacto Muito Alto

- Bugs do robô afetarem o fluxo de pacientes e colaboradores no hospital
    - Explicação: Se o robô apresentar falhas de software ou travamentos, isso pode interromper a movimentação de pessoas, afetando a eficiência do hospital e gerando atrasos nas rotinas de trabalho.

    - Mitigação:
        - Testes extensivos e simulações: Realizar testes em ambientes controlados antes da implementação no hospital para identificar e corrigir bugs.
        - Monitoramento em tempo real: Implementar sistemas de monitoramento que detectem erros e permitam correções rápidas.
        - Definir planos de contingência para assistência manual aos pacientes em caso de falhas.

- Robô responder algo indevido durante a conversa com o usuário

    - Explicação: o robô pode responder algo indevido, incorreta ou ofensiva ao usuário. Isso por acontecer devido a falhas no processamento de linguagem ou falta de filtros apropriados, o que afeta a experiência do usuário e prejudica a reputação do sistema, especialmente ao lidar com públicos vulneráveis.

    - Mitigação:
        - Filtros de conteúdo: Integrar filtros que detectem e bloqueiem respostas potencialmente ofensivas, inadequadas ou sem sentido. O uso de listas negras de termos e expressões inadequadas pode prevenir que o robô gere esse tipo de conteúdo.
        - Ajuste de parâmetros: Limitar a capacidade do robô de gerar respostas abertas e manter as interações mais estruturadas e previsíveis em casos de maior sensibilidade, como atendimento a populações vulneráveis.
        - Atualização contínua: Manter o sistema atualizado com feedback de usuários, revisando os cenários onde o robô pode falhar para melhorar sua capacidade de lidar com situações complexas e sensíveis.

### Riscos de Probabilidade Baixa e Impacto Muito Alto

- Comprometimento de dados sensíveis

    - Explicação: Informações confidenciais sobre pacientes podem ser armazenadas temporariamente durante a navegação, o que pode causar problemas de privacidade e segurança.

    - Mitigação:
        - Criptografia de dados: Implementar criptografia de ponta a ponta para proteger as informações nos sistemas digitais.
        - Autenticação e controle de acesso: Utilizar autenticação robusta para garantir que apenas pessoal autorizado tenha acesso aos dados pessoais dos pacientes.
        - Auditorias regulares de segurança: Realizar auditorias periódicas para garantir que as medidas de segurança estão atualizadas e eficazes.

### Riscos de Probabilidade Moderada e Impacto Alto
- Problemas na comunicação entre robô e pacientes

    - Explicação: Há o risco de o robô não interpretar corretamente as solicitações dos pacientes, resultando em orientações incorretas.

    - Mitigação:
        - Testes contínuos da interface de comunicação do robô, incluindo feedback dos usuários.
        - Aperfeiçoamento do processamento de linguagem natural para melhorar a compreensão do robô.

### Riscos de Probabilidade Moderada e Impacto Baixo

- Alto custo do projeto dado às tecnologias necessárias
    - Explicação: As tecnologias envolvidas, como navegação autônoma, integração de sistemas e segurança, podem elevar significativamente os custos do projeto. O custo inicial pode ser um obstáculo para implementação em larga escala.

    - Mitigação:
        - Análise de custo-benefício detalhada: Avaliar o impacto financeiro e o retorno sobre investimento (ROI) a longo prazo, priorizando tecnologias essenciais.
        - Procurar financiamento ou parcerias: Buscar parcerias com empresas de tecnologia ou subsídios para reduzir o impacto financeiro do projeto.
        - Escalabilidade gradual: Implementar o projeto em fases, escalando conforme os resultados positivos são observados.

## Oportunidades

&emsp;As seguintes oportunidades foram identificadas para o projeto de um robô autônomo que utiliza IA, cada uma posicionada na matriz de acordo com sua probabilidade e impacto. Também foi desenvolvido um plano de ação para lidar com os riscos caso eles se tornem realidade.

### Oportunidades de Probabilidade Alta e Impacto Muito Alto

- Melhoria na experiência do paciente
    - Explicação: O robô pode melhorar a experiência de navegação no hospital, especialmente para visitantes que não conhecem o local, otimizando o fluxo de pessoas.

    - Plano de Ação para Riscos:
        - Implementação de mapas e rotas claras, com atualizações em tempo real.
        - Treinamento de funcionários para intervenções rápidas em caso de falhas.

### Oportunidades de Probabilidade Moderada e Impacto Muito Alto

- O projeto pode ser escalado para outros hospitais, abrindo novos mercados.

    - Explicação: Uma vez validado, o projeto pode ser facilmente adaptado e escalado para outros hospitais ou setores que precisem de automação, criando novas oportunidades de negócios e receitas em mercados maiores.

    - Plano de Ação para Riscos:
        - Padronização e personalização: Criar uma versão padronizada do robô que possa ser facilmente personalizada para diferentes ambientes hospitalares, minimizando os ajustes necessários durante a expansão.
        - Parcerias estratégicas: Formar parcerias com outras instituições de saúde para acelerar a implantação em novos mercados e garantir suporte local.

### Oportunidades de Probabilidade Alta e Impacto Alto

- Automação de tarefas simples e repetitivas

    - Explicação: O robô pode aliviar os funcionários de tarefas como dar direções ou guiar pacientes, permitindo que eles se concentrem em outras atividades.

    - Plano de Ação para Riscos:
        - Manutenção preventiva do robô para garantir que esteja sempre pronto para atender pacientes.
            - Criação de rotas de backup caso o robô precise ser substituído temporariamente.

### Oportunidades de Probabilidade Moderada e Impacto Alto

- Aprimoramento do Marketing e Imagem do Hospital

    - Explicação: A introdução de robôs inteligentes demonstra inovação tecnológica e pode ser um fator de atração para pacientes e visitantes, projetando uma imagem moderna e eficiente do hospital.

    - Plano de Ação para Riscos:
        - Estratégia de comunicação: Investir em campanhas de divulgação sobre a tecnologia de robôs guias no hospital, destacando os benefícios para a experiência do paciente.
        - Estudo de caso: Documentar a implementação e resultados do uso do robô para atrair mais pacientes e potenciais parceiros.

### Oportunidades de Probabilidade Moderada e Impacto Moderado

- Redução do Tempo de Espera nas Recepções

    - Explicação: O robô pode diminuir o congestionamento nas recepções e balcões de informações ao fornecer instruções automáticas de maneira eficiente e em tempo real.

    - Plano de Ação para Riscos:
        - Integração com o sistema de fila: O robô pode se conectar ao sistema de filas do hospital, oferecendo um serviço de check-in automatizado e melhorando a organização de atendimento.
        - Testes regulares de uso: Garantir que o sistema de comunicação do robô funcione perfeitamente em horários de pico.

- Falhas de hardware no TurtleBot

    - Explicação: Problemas com componentes físicos do robô, como motores ou sensores, podem interromper seu funcionamento. Essas falhas podem resultar em paradas inesperadas ou mau desempenho, prejudicando a operação contínua e confiável do robô no hospital.

    - Mitigação:
        - Manutenção preventiva: Realizar verificações regulares e preventivas no hardware do robô para identificar problemas antes que afetem as operações.
        - Peças de reposição disponíveis: Manter um inventário de peças de reposição críticas para reparos rápidos em caso de falhas.
        - Redundância: Ter robôs de backup prontos para substituir qualquer robô que apresente problemas de hardware.

## Conclusão

&emsp;A Matriz de Riscos ajudou a identificar ameaças como falhas de navegação, bugs e questões de segurança de dados, com planos de mitigação bem definidos. Ao mesmo tempo, as oportunidades, como a automação de tarefas repetitivas, otimização fluxo de pessoas no hospital e expansão para outros hospitais, destacam o potencial do projeto para melhorar a eficiência e segurança no ambiente hospitalar.

&emsp;Para o sucesso deste projeto, é crucial focar em rigorosos testes de navegação e comunicação com os pacientes, além de estabelecer um plano de contingência para minimizar falhas operacionais. A expansão para outros mercados pode ser facilitada por meio de parcerias estratégicas com outras instituições de saúde, ao mesmo tempo que a customização do robô deve ser prioridade para diferentes ambientes hospitalares.





