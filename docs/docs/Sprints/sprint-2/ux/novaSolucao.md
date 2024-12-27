---
title: Atualização da Solução
sidebar_position: 1
slug: "/solucao2"
---

## O que mudou

&emsp;A partir da conversa com o parceiro durante a sprint review na semana 2, o grupo sentiu necessidade de alterar partes da solução proposta, devido a implicações técnicas e à maneira como o produto agregaria valor no contexto analisado, do ponto de vista de negócios. Dessa forma, a solução foi adaptada para proporcionar uma melhor experiência aos colaboradores e pacientes do hospital, agregando mais valor por meio do uso de processamento de linguagem natural e da navegação autônoma do robô.

## Nova solução

&emsp;A nova solução busca melhorar a experiência do paciente durante o período de internação e facilitar o cotidiano da enfermaria com diversos pedidos acerca de hotelaria e restaurantes do hospital. Assim, com um chatbot do WhatsApp, a pessoa enfermeira pode fazer pedidos de refeições, solicitar a limpeza do quarto, pedir ajuda para o paciente e solicitar insumos não medicamentosos, como luvas e toucas. A partir desses pedidos enviados para as alas respectivas, um robô autônomo será direcionado para a ala para retirar o pedido e transportá-lo até o quarto do paciente ou área destinada. Assim, ao chegar no local, o enfermeiro retira o pedido do robô e o robô se direciona para o próximo pedido, até que todos sejam entregues. Após finalizar seu trabalho, o robô retorna para a base de carregamento.

&emsp;Essa solução tem como objetivo facilitar o cotidiano da enfermaria, trazendo mais agilidade e eficiência para os pedidos dos pacientes e enfermeiros. Além disso, o robô autônomo oferece um diferencial de agilidade e autonomia para a entrega dos pedidos, trazendo mais valor para o hospital e para a experiência do paciente, que já está em um momento delicado e precisa de atenção e cuidado.

## O que a solução contempla

- Comunicação por meio de um chatbot no *WhatsApp* para pedidos de refeições, limpeza do quarto, ajuda para o paciente e insumos não medicamentosos feitos por enfermeiros. Para os pedidos de transporte, o *chatbot* envia comandos para que o robô inicie a entrega.
- Robô autônomo para retirar os pedidos e transportar até o quarto do paciente.
- Entrega dos pedidos de forma autônoma e eficiente, com coleta de dados e exibição das colaborações do robô em um *dashbord*.
- Retorno do robô para a base de carregamento após finalizar o trabalho.

## O que a solução não contempla

- Entrega de medicamentos: por questões de segurança do paciente e de controle da enfermaria, não cabe ao robô realizar a entrega de medicamentos.
- Entrega de pedidos para áreas externas do hospital: por limitações técnicas de teste da prova de conceito, não é possível testar em áreas externas ou não controladas.
- Entrega de pedidos para áreas de emergência: entende-se que, para fins de teste da prova de conceito e para garantir a segurança do paciente, não cabe à solução entregar pedidos para áreas de emergência.
- Sistema de pedidos dos pacientes: pode ser uma funcionalidade para etapas futuras, mas visando uma entrega mínima mais completa e testável, foi decidido focar nos pedidos realizados pela enfermaria.
- Sistema de pedidos para a cozinha: considera-se que a cozinha já possui um sistema de pedidos; o robô apenas retira os pedidos e os entrega para a enfermaria, integrando o sistema existente da cozinha ao chatbot.