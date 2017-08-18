# ID da mensagem. Afim de ser utilizado como contra-prova para verificar se a comunicação está sincronizada
uint8 id

# Booleano que define se o time é amarelo ou azul
bool is_team_yellow

# Lista dinâmica de comandos. Sendo sempre 3, devido a quantidade robôs
c_robot_command[3] robot_commands

# Situação em que se encontra o comando. No futuro os robôs serão posicionados autonomamente
uint8 situation

# Nome do time
string name
