# Essa informação é utilizada para
# representar uma bola no espaço do Futebol de Robôs, contendo: Pose; Velocidade linear e tangencial; Estimativa
# da Pose por parte do Filtro de Kalman (Necessário em ambiente real pelo VSS-Vision); e Velocidade linear e tangencial
# estimados pelo Filtro de Kalman.

# Posição de uma bola. Sendo somente esse o valor OBRIGATÓRIO, pois os outros estão sendo implementados ainda.
s_pose pose

# Velocidade linear e tangencial de uma bola
s_pose v_pose

# Estimativa de Posição de uma bola
s_pose k_pose

# Estimativa de Velocidade linear e tangencial de uma bola
s_pose k_v_pose
