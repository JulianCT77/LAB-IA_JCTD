from cuenta_bancaria import CuentaBancaria

def mostrar_menu():
    """Muestra el menú de opciones"""
    print("\n--- MENÚ BANCARIO ---")
    print("1. Depositar")
    print("2. Retirar")
    print("3. Ver saldo")
    print("4. Salir")

def main():
    # Crear una cuenta bancaria
    titular = input("Ingrese el nombre del titular: ")
    saldo_inicial = float(input("Ingrese el saldo inicial: $"))
    cuenta = CuentaBancaria(titular, saldo_inicial)
    
    # Bucle principal del programa
    while True:
        mostrar_menu()
        opcion = input("Seleccione una opción (1-4): ")
        
        if opcion == '1':
            monto = float(input("Monto a depositar: $"))
            cuenta.depositar(monto)
            
        elif opcion == '2':
            monto = float(input("Monto a retirar: $"))
            cuenta.retirar(monto) #Cuanta porque va a la classe y leugo a la funcion 
            
        elif opcion == '3':
            cuenta.mostrar_saldo()
        elif opcion == '4':
            print("\nGracias por usar nuestro servicio bancario. ¡Hasta pronto!")
            break
        else:
            print("Opción no válida. Por favor ingrese 1, 2, 3 o 4.")

if __name__ == "__main__":
    main()