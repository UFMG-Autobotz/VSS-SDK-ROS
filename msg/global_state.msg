# ID da mensagem, pode ser enviado como contra-prova para verificar se a comunicação está sincronizada. Sendo este o único valor OPICIONAL
uint8 id

# Booleano que define a origem da mensagem. Caso TRUE a mensagem vem do VSS-Vision, caso FALSE a mensagem vem do VSS-Simulator
bool origin

# Lista dinâmica de bolas. Esta parte é definida dinâmicamente, pois podem existir situações indesejadas onde mais de uma bola está presente no ambiente
# REVIEW Mudei para apenas uma bola porque tanto o codigo do viewer quanto do simulator so enviavam uma bola
s_ball_state ball

# Lista dinâmica de robôs amarelos. Em geral sempre são enviados 3 robôs.
s_robot_state[3] robots_yellow

# Lista dinâmica de robôs azuis. Em geral sempre são enviados 3 robôs.
s_robot_state[3] robots_blue

# Situação em que a partida se encontra, será utilizado no futuro, para "obrigar" os robôs sempre a se reposicionarem autonomamente no campo
uint8 situation
  # SITUATIONS
  # 0 = Normal Game
  # 1 = Re-organize (GOAL_TEAM1)
  # 2 = Re-organize (GOAL_TEAM2)
  # 3 = Re-organize (FAULT_TEAM1)
  # 4 = Re-organize (FAULT_TEAM2)
  # 5 = Re-organize (PENALTY_TEAM1)
  # 6 = Re-organize (PENALITY_TEAM2)
  # 99 = Finish

# Tempo: 1, 2, 3 ou 4
uint8 time

# Quantidade de Gols do Time Amarelo
uint8 goals_yellow

# Quantidade de Gols do Time Azul
uint8 goals_blue

# Nome do time amarelo
string name_yellow

# Nome do time azul
string name_blue
