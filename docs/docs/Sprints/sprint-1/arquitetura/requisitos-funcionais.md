---
title: Requisitos funcionais
slug: "/requisitos-funcionais"
---

# Requisitos Funcionais

A seção de **Requisitos Funcionais** descreve as funcionalidades essenciais que o robô deve ter para cumprir seu propósito no ambiente hospitalar. Cada requisito é associado a uma user story, garantindo que as necessidades dos pacientes e dos técnicos de manutenção sejam atendidas de maneira eficiente. Os critérios de aceitação e os casos de teste fornecem uma base clara para verificar o correto funcionamento de cada funcionalidade, garantindo que o sistema opere de maneira segura, intuitiva e adaptável a diferentes cenários hospitalares.

| Requisito | Descrição                                                                                                                                                                                  | User Story Relacionada | Critérios de Aceitação                                                                                        | Caso de Teste                                                                                                        |
| --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------- | ------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| **RF01**  | O robô deverá se locomover autonomamente pelos corredores do hospital utilizando SLAM, ajustando sua rota conforme obstáculos dinâmicos (como pessoas e objetos).                          | US01 (Paciente)        | O robô navega de forma autônoma até o destino final, ajustando sua rota em tempo real para evitar obstáculos. | Configurar um destino e verificar se o robô guia o paciente sem colisões, desviando de obstáculos.                   |
| **RF02**  | O robô deve guiar o paciente até o consultório correto, ajustando sua rota conforme necessário.                                                                                            | US01 (Paciente)        | O robô navega até o destino correto sem erros de trajeto.                                                     | Definir o destino e verificar se o robô guia o paciente até o consultório.                                           |
| **RF03**  | O robô deve apresentar informações sobre o hospital, como localização de banheiros e refeitórios, ao longo do caminho.                                                                     | US02 (Paciente)        | O robô exibe informações sobre os pontos principais do hospital durante a navegação.                          | Verificar se o robô apresenta corretamente as informações ao longo do trajeto.                                       |
| **RF04**  | O robô deve permitir a confirmação do destino na tela antes de iniciar a navegação.                                                                                                        | US03 (Paciente)        | O robô exibe uma tela de confirmação antes de iniciar a navegação, aguardando a validação do paciente.        | Configurar um destino e verificar se o robô aguarda confirmação antes de iniciar a rota.                             |
| **RF05**  | O sistema de comandos de voz deve ser simples, permitindo que o paciente siga as instruções sem dificuldades. O robô também pode interagir de maneira amigável, fazendo perguntas simples. | US04 (Paciente)        | O robô entende e executa comandos de voz simples e interage de forma amigável com o paciente.                 | Emitir comandos de voz e verificar se o robô responde corretamente e interage com o paciente.                        |
| **RF06**  | O robô deve ser acessível diretamente da recepção, permitindo que o paciente o chame sem ajuda.                                                                                            | US05 (Paciente)        | O robô responde a comandos de chamada da recepção de forma autônoma.                                          | Verificar se o robô é chamado com sucesso a partir da recepção.                                                      |
| **RF07**  | O robô deve enviar notificações em tempo real sobre possíveis erros para o técnico de manutenção.                                                                                          | US01 (Técnico)         | O sistema envia alertas automáticos ao técnico quando detecta erros ou problemas.                             | Simular um erro e verificar se a notificação é enviada corretamente ao técnico.                                      |
| **RF08**  | O técnico deve poder acessar remotamente o diagnóstico do robô para solucionar problemas.                                                                                                  | US02 (Técnico)         | O técnico pode acessar diagnósticos e tomar decisões remotamente.                                             | Simular um erro e verificar se os dados de diagnóstico são acessíveis remotamente.                                   |
| **RF09**  | O robô deve ter rotinas automáticas de monitoramento, verificando seu estado de saúde periodicamente.                                                                                      | US03 (Técnico)         | O sistema de monitoramento automático coleta e reporta dados sobre o robô regularmente.                       | Configurar a rotina automática e verificar o estado do robô periodicamente.                                          |
| **RF10**  | O robô deve ser integrado com o painel de controle hospitalar para centralizar informações de desempenho.                                                                                  | US04 (Técnico)         | O robô reporta seu desempenho ao painel de controle hospitalar.                                               | Verificar a integração e se os dados do robô são recebidos no painel de controle.                                    |
| **RF11**  | O robô deve ter baixa necessidade de recarga manual, operando por longos períodos sem intervenção.                                                                                         | US05 (Técnico)         | O robô notifica quando a recarga é necessária, operando por longos períodos sem intervenção manual.           | Monitorar a operação do robô por longas sessões e verificar a necessidade de recarga.                                |
| **RF12**  | Deve ser possível teleoperar o robô remotamente, permitindo controle em situações excepcionais.                                                                                            | US04 (Técnico)         | O robô pode ser operado remotamente em qualquer ponto do hospital.                                            | Simular a teleoperação e verificar o controle remoto do robô.                                                        |
| **RF13**  | A interface web no tablet/celular deve permitir ao usuário operar o robô sem o uso de voz, acessando as mesmas funcionalidades básicas.                                                    | US03 (Paciente)        | A interface web permite controle completo do robô (navegação, escolha de destinos) sem comandos de voz.       | Verificar se o tablet/celular permite a operação completa do robô sem utilizar comandos de voz.                      |
| **RF14**  | O sistema deve ser projetado para funcionar em diferentes layouts hospitalares, com suporte a múltiplos mapas ou ajustes em tempo real.                                                    | US01 (Paciente)        | O robô consegue navegar em diferentes layouts hospitalares, adaptando-se a mapas diferentes.                  | Testar o robô em diferentes layouts de hospitais e verificar a adaptação.                                            |
| **RF15**  | O robô não deve lidar com dados sensíveis, mas deve garantir a segurança física dos pacientes, evitando colisões e garantindo operações seguras.                                           | US01, US05 (Paciente)  | O robô realiza navegação segura, sem colisões e garantindo a integridade física dos pacientes.                | Simular trajetos com obstáculos e verificar a segurança física do paciente durante a navegação.                      |
| **RF16**  | O sistema deve permitir integração via API para acessar e se conectar a outros sistemas hospitalares.                                                                                      | US04 (Técnico)         | O robô é capaz de se integrar com outros sistemas via API, trocando informações conforme necessário.          | Verificar se o robô consegue realizar trocas de dados e se integrar com outros sistemas hospitalares através de API. |