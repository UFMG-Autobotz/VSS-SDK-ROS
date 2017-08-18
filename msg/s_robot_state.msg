# Essa informação é utilizada para
# representar um robos no espaço do Futebol de Robôs, contendo: Pose; Velocidade linear, tangencial e angular;
# Estimativa da Pose por parte do Filtro de Kalman (Necessário em ambiente real pelo VSS-Vision); e Velocidade linear,
# tangencial e angular estimados pelo Filtro de Kalman.

# Posição e Orientação de um robô. Sendo somente esse o valor OBRIGATÓRIO, pois os outros estão sendo implementados ainda.
s_pose pose

# Velocidade linear, tangencial e angular de um robô
s_pose v_pose

# Posição e Orientação de um robô estimados pelo filtro de kalman
s_pose k_pose

# Velocidade linear, tangencial e angular estimados pelo filtro de kalman
s_pose k_v_pose

# Cor da etiqueta secundária de um robô (Somente VSS-Vision)
s_rgb color
