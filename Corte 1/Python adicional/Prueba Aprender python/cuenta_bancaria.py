

class CuentaBancaria:
    """
    Clase que representa una cuenta bancaria con operaciones básicas
    """
    
    def __init__(self, titular, saldo_inicial=0):
        """
        Inicializa una nueva cuenta bancaria
        
        Args:
            titular (str): Nombre del titular de la cuenta
            saldo_inicial (float): Saldo inicial de la cuenta (por defecto 0)
        """
        self.titular = titular
        self.saldo = saldo_inicial
    
    def depositar(self, monto):
        """
        Deposita dinero en la cuenta
        
        Args:
            monto (float): Cantidad a depositar (debe ser positiva)
        """
        if monto > 0:
            self.saldo += monto
            print(f"Depositado ${monto:.2f}. Saldo actual: ${self.saldo:.2f}")
        else:
            print("Error: El monto debe ser positivo")
    
    def retirar(self, monto):
        """
        Retira dinero de la cuenta
        
        Args:
            monto (float): Cantidad a retirar (debe ser positiva y <= saldo)
        """
        if 0 < monto <= self.saldo:
            self.saldo -= monto
            print(f"Retirado ${monto:.2f}. Saldo actual: ${self.saldo:.2f}")
        else:
            print("Error: Fondos insuficientes o monto inválido")
    
    def mostrar_saldo(self):
        """
        Muestra el saldo actual de la cuenta
        """
        print(f"Saldo actual de {self.titular}: ${self.saldo:.2f}")