from transitions import Machine

states = ['Conectado', 'Desconectado', 'Error', 'Graficando']

transitions = [
    {'trigger': 'Conectar', 'source': 'Desconectado', 'dest': 'Conectado'},
    {'trigger': 'Desconectar', 'source': 'Conectado', 'dest': 'Desconectado'},
    {'trigger': 'Conexión Fallida', 'source': 'Conectado', 'dest': 'Error'},
    {'trigger': 'Reset', 'source': 'Error', 'dest': 'Desconectado'},
    {'trigger': 'Graph', 'source': 'Conectado', 'dest': 'Graficando'},
    {'trigger': 'NoGraph', 'source': 'Graficando', 'dest': 'Conectado'},
    {'trigger': 'NoGraphForced', 'source': 'Graficando', 'dest': 'Desconectado'},
]

USARTstate = Machine(states = states, transitions=transitions, initial = 'Desconectado')