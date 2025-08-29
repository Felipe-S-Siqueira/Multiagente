# Multiagente — Exploração em Grade

Este projeto é um sistema multiagente simples em Python, onde robôs exploradores constroem coletivamente um mapa de um ambiente 2D (grade 10x10).

## Funcionalidades

* Gera aleatoriamente um ambiente 10x10 com obstáculos, áreas livres e pontos de interesse (POIs).
* Inicializa 3 ou mais robôs em posições distintas.
* Cada robô percebe células vizinhas e atualiza o mapa global com as informações coletadas.
* Robôs exploram o ambiente de forma inteligente, evitando revisitar células recentes e priorizando POIs desconhecidos.
* Exibe a exploração em tempo real usando uma animação com `matplotlib`.

## Requisitos

* Python 3.8 ou superior
* Bibliotecas: `numpy`, `matplotlib`

Instale as dependências com:

```bash
pip install numpy matplotlib
```

## Como executar

1. Abra o terminal na pasta do projeto.
2. Execute o arquivo principal:

```bash
python src/multiagent_compact.py
```

3. A animação mostrará os robôs explorando o ambiente e o mapa global sendo construído em tempo real.

## Interpretação visual

* **Vermelho**: Obstáculos
* **Branco**: Áreas livres
* **Verde**: Pontos de interesse (POIs)
* **Cinza claro**: Células desconhecidas no mapa global

## Personalização

Você pode ajustar no início do arquivo `multiagent_compact.py`:

* `GRID_SIZE`: tamanho da grade
* `NUM_OBSTACLES`: número de obstáculos
* `NUM_POIS`: número de pontos de interesse
* `NUM_ROBOTS`: quantidade de robôs
* `STEPS`: número de passos da simulação
* `RANDOM_SEED`: definir uma semente para reprodutibilidade ou `None` para aleatório
