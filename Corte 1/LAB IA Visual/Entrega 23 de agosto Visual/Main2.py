from Entrega2 import InvertedPendulumSimulator2

def main():
    # Crea la instancia del simulador
    simulator = InvertedPendulumSimulator2()
    
    # Pregunta si quiere continuar
    respuesta = input("¿Quieres continuar? (si/no): ").lower()
    
    if respuesta == 'no' or respuesta == 'n':
        print("Saliendo del programa...")
        return
    
    # Inicia la simulación
    simulator.run()

if __name__ == "__main__":
    main()