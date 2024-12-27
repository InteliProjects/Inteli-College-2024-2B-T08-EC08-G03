# CLI de Navegação do TurtleBot com ROS 2

Este projeto facilita o controle e a navegação do TurtleBot utilizando ROS 2, oferecendo uma solução inicial robusta. A utilização do **Nav2** permite que o robô realize navegações autônomas, baseadas em mapeamento e planejamento de trajetórias. O **Docker Compose** foi adotado para garantir que todas as dependências e configurações do ambiente sejam consistentes e facilmente replicáveis, facilitando o desenvolvimento.

### Impacto no Projeto

A integração destas ferramentas proporciona os seguintes benefícios:

- **Autonomia do Robô**: O Nav2 permite que o TurtleBot3 realize navegação autônoma em ambientes já mapeados, possibilitando a marcação e navegação automática entre pontos de interesse. Atualmente a seleção dos pontos é feita manualmente no robô, mas planejamos integrar esta funcionalidade ao backend para melhorar a experiência do usuário. 
- **Consistência do Ambiente**: O uso do Docker Compose assegura que todos os desenvolvedores e operadores utilizem o mesmo ambiente, reduzindo problemas de compatibilidade e simplificando o processo de configuração. Isso é de extrema importancia quando utilizamos ROS,   especialmente no nosso caso, onde temos que usar o especificamente o ROS 2 humble com o ubuntu na versão 22.04. A utilização do ROS 2 Humble é necessária em nosso projeto pois é nesta distribuição que encontramos os pacotes do Nav2 compatíveis com o Turtlebot3 Burger, nosso robô de desenvolvimento.


## Estrutura do Projeto

O projeto está organizado nos seguintes diretórios principais:

- `src/cli/`: Contém os scripts de navegação do TurtleBot.
- `src/robot/`: Contém a configuração Docker necessária para executar os scripts em um ambiente containerizado.

## Requisitos

Antes de iniciar, certifique-se de que os seguintes pré-requisitos estão atendidos:

- **Sistema Operacional**: Ubuntu (preferencialmente 22.04 ou superior) ou qualquer sistema com suporte ao Docker.
- **Docker**: Instalado e configurado no sistema. [Instalação do Docker](https://docs.docker.com/get-docker/)
- **Docker Compose**: Instalado no sistema. [Instalação do Docker Compose](https://docs.docker.com/compose/install/)
- **Permissões de Docker**: O usuário atual deve ter permissões para executar comandos Docker sem `sudo`.

## Instalação

### 1. Clonar o Repositório do Projeto

Clone o repositório e navegue até o diretório do projeto:

```bash
git clone https://github.com/Inteli-College/2024-2B-T08-EC08-G03.git
cd 2024-2B-T08-EC08-G03
```

## Uso

### Etapas

#### 1. Conectar ao Robô via SSH e Executar o Bringup

Conecte-se ao robô via SSH e execute o comando de bringup para iniciar os nós essenciais do TurtleBot:

```bash
ssh usuario@ip_do_robo
ros2 launch turtlebot3_bringup robot.launch.py
```

_Substitua `usuario` e `ip_do_robo` pelas credenciais e endereço IP do seu robô._

#### 2. Configurar o Docker no Computador

No computador onde o repositório foi clonado, navegue até `src/robot` e construa os serviços Docker Compose:

```bash
cd src/robot
docker compose build
```

#### 3. Executar a CLI

Utilize o Docker Compose para executar o script de navegação dentro do container Docker:

**Navegação para Posições Pré-definidas**:

```bash
docker compose run overlay "python cli/main.py select_pose"
```

**Nota**: Certifique-se de que o script `main.py` e o arquivo `poses.txt` estejam presentes no workspace `/overlay_ws/src/cli` dentro do container Docker.
