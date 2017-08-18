# Essa mensagem é uma submensagem do pacote global_debug que de fato é enviado. Essa informação é utilizada para
# represetar um corpo rígido (objeto: robô ou bola) em um espaço do Futebol de Robôs, sendo necessário apenas:
# posição X e Y e rotação em Z. O ID é necessário para que se saiba de qual robô é a pose.

uint8 id
float32 x
float32 y
float32 yaw
