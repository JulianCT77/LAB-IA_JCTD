#julian Camilo Torres

from Entrega3 import FuzzyPendulumController

def main():
    # Crea la instancia del simulador
    simulator = FuzzyPendulumController()
    
    # Pregunta si quiere continuar
    respuesta = input("¿Quieres continuar? (si/no): ").lower()
    
    if respuesta == 'no' or respuesta == 'n':
        print("Saliendo del programa...")
        return
    
    # Inicia la simulación
    simulator.run()

if __name__ == "__main__":
    main()