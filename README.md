# Multiagente — Exploração em Grade

Este projeto implementa um sistema multiagente em Python, onde robôs exploradores constroem coletivamente um mapa de um ambiente 2D (grade 10x10).

## Funcionalidades

- Gera aleatoriamente um ambiente 10x10 com obstáculos, áreas livres e pontos de interesse (POIs).  
- Inicializa 3 ou mais robôs em posições distintas.  
- Cada robô percebe células vizinhas e atualiza o mapa global com as informações coletadas.  
- Robôs exploram o ambiente de forma inteligente, evitando revisitar células recentes e priorizando POIs desconhecidos.  
- Exibe a exploração em tempo real através de uma animação com `matplotlib`.  

## Como funciona

1. **Ambiente (`Environment`)**: cria o mapa com células livres, obstáculos e pontos de interesse.  
2. **Robôs (`RoboAgente`)**: cada robô tem posição, memória de células visitadas, mapa local e plano de movimento.  
3. **Percepção**: os robôs percebem células adjacentes e compartilham informações no mapa global.  
4. **Planejamento**: cada robô busca o caminho mais curto até uma célula desconhecida ou ponto não visitado.  
5. **Movimento e conflitos**: os robôs escolhem movimentos baseados em prioridade e resolvem conflitos caso dois robôs tentem ocupar a mesma célula.  
6. **Mapa global**: atualizado continuamente com as informações de todos os robôs.  
7. **Visualização**: a simulação é animada com `matplotlib`, mostrando o progresso da exploração e os pontos encontrados.  

## Requisitos

- Python 3.8 ou superior  
- Bibliotecas: `numpy`, `matplotlib`  

Instale as dependências com:

```bash
pip install numpy matplotlib
Como executar
Abra o terminal na pasta do projeto.

Execute o arquivo principal:

bash
Copiar código
python src/multiagent_compact.py
A animação mostrará os robôs explorando o ambiente e o mapa global sendo construído passo a passo.

Interpretação visual
Vermelho: Obstáculos

Branco: Áreas livres

Verde: Pontos de interesse (POIs)

Cinza claro: Células desconhecidas no mapa global

Bolhas coloridas: posição dos robôs (cada cor representa um robô diferente)

Personalização
Você pode ajustar no início do arquivo multiagent_compact.py:

GRID_SIZE: tamanho da grade

NUM_OBSTACLES: número de obstáculos

NUM_POINTS: número de pontos de interesse

NUM_ROBOTS: quantidade de robôs

STEPS: número de passos da simulação

RANDOM_SEED: definir uma semente para reprodutibilidade ou None para aleatório

Resultados
Ao final da simulação, o programa imprime:

O mapa original (ground truth) com obstáculos e pontos de interesse.

O mapa global construído pelos robôs durante a exploração.

Os pontos de interesse encontrados pelos robôs.