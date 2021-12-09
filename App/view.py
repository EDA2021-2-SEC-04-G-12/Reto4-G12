"""
 * Copyright 2020, Departamento de sistemas y Computación, Universidad
 * de Los Andes
 *
 *
 * Desarrolado para el curso ISIS1225 - Estructuras de Datos y Algoritmos
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along withthis program.  If not, see <http://www.gnu.org/licenses/>.
 """

import config as cf
import sys
import controller
from DISClib.ADT import list as lt
from DISClib.ADT import map as mp
assert cf


"""
La vista se encarga de la interacción con el usuario
Presenta el menu de opciones y por cada seleccion
se hace la solicitud al controlador para ejecutar la
operación solicitada
"""

def printMenu():
    print("Bienvenido")
    print("1- Iniciar Analizador")
    print("2- Cargar información de los aeropuertos y rutas")
    print("3- Requerimiento 1: Encontrar puntos de interconexión aérea")
    print("4- Requerimiento 2: Encontrar clústeres de tráfico aéreo")
    print("5- Requerimiento 3: Encontrar la ruta más corta entre ciudades")
    print("6- Requerimiento 4: Utilizar las millas de viajero")
    print("7- Requerimiento 5: Cuantificar el efecto de un aeropuerto cerrado")
    print("8- Salir del programa")


catalog = None

def printCityInfo (cityinfo):
    city = cityinfo['city']
    country = cityinfo['country']
    lat = cityinfo['lat']
    lng = cityinfo['lng']
    population = cityinfo['population'] 
    print(f"| {city:9}| {country:9}| {lat:9}| {lng:9}| {population:9}|")
    print("+" + "-"*10 +"+" + "-"*10 + "+" + "-"*10 + "+" + '-'*10 + '+' + "-"*10 + "+")
def printAirportInfo (airportinfo):
    """
    Impresion de datos requerimiento 1
    """
    Name = airportinfo['info']['Name']
    City = airportinfo['info']['City']
    Country = airportinfo['info']['Country']
    IATA = airportinfo['info']['IATA']
    conections = airportinfo['numConexions'] 
    inbound = airportinfo['inbound']
    outbound = airportinfo['outbound']
    print(f"| {Name:31}| {City:18}| {Country:22}| {IATA:7}| {conections:14}| {inbound:11}| {outbound:11}|")
    print("+" + "-"*32 +"+" + "-"*19 + "+" + "-"*23 + "+" + '-'*8 + '+' + "-"*15 + "+" + "-"*12 + '+' + '-'*12 + '+')

def printRoute (paso): #TODO:PRINT REQ 3
    departure = paso['vertexA']
    destination = paso['vertexB']
    distance = paso['weight']
    print(f"| {departure:13}| {destination:13}| {distance:15}|")
    print("+" + "-"*14 +"+" + "-"*14 + "+" + "-"*16 + "+")

def displayCities(city) : 
    citymap = controller.cityMap(analyzer,city)
    print('Se hallaron varias ciudades con el mismo nombre, por favor elija una: ')
    i = 1 
    for city in lt.iterator(citymap) : 
        print(f"{i}. {city}")
        i += 1 
    seleccion = input("Ingrese una opcion: ")
    elemento = lt.getElement(citymap,int(seleccion))
    return elemento
    
def printArbolExpansion(mst):
    departure = mst['vertexA']
    destination = mst['vertexB']
    distance = mst['weight']
    print(f"| {departure:13}| {destination:13}| {distance:15}|")
    print("+" + "-"*14 +"+" + "-"*14 + "+" + "-"*16 + "+")


    
    
"""
Menu principal
"""
while True:
    printMenu()
    inputs = input('Seleccione una opción para continuar\n')
    if int(inputs[0]) == 1:
        print("\nInicializando....")
        analyzer = controller.init()

    elif int(inputs[0]) == 2:
        print("Cargando datos .... ")
        services = controller.loadServices(analyzer)
        Num_1 = services[1]
        Num_2 = services[2]
        Num_3 = services[3]
        Num_4 = services[4]
        Num_5 = services[5]
        Num_6 = services[6]
        Num_7 = services[7]
        Num_8 = services[9]
        Num_9 = services[8]
        print('------------------------------------------------------------------------------------------------------------------------------------------')
        print('Grafo 1')
        print(f'Total de aeropuertos = {Num_1}')
        print(f'Total de rutas aéreas = {Num_2}')
        print(Num_6)
        print(Num_7)
        print('------------------------------------------------------------------------------------------------------------------------------------------')
        print('Grafo 2')
        print(f'Total de aeropuertos = {Num_3}')
        print(f'Total de rutas aéreas = {Num_4}')
        print('------------------------------------------------------------------------------------------------------------------------------------------')
        print('Ciudades')
        print(f'Total de ciudades = {Num_5}')
        print("La primera y última ciudad cargada son: ")
        print("+" + "-"*10 +"+" + "-"*10 + "+" + "-"*10 + "+" + '-'*10 + '+' + "-"*10 + "+")
        print("| " + 'Ciudad' + ' '*3 + "| " + 'País' + ' '*5 + "| " + 'Latitud' + ' '*2 + "| " + 'Longitud'+" "*2 +"|"+"Población" +' ' +"|")
        print("+" + "-"*10 +"+" + "-"*10 + "+" + "-"*10 + "+" + '-'*10 + '+' + "-"*10 + "+")
        printCityInfo(Num_8)
        printCityInfo(Num_9)

    elif int(inputs[0]) == 3:
        print("Calculando ... ")
        aeropuertos = controller.masConectados(analyzer)
        conected = lt.size(aeropuertos)
        print(f"Numero de aeropuertos conectados: {conected}\n")
        print("+" + "-"*32 +"+" + "-"*19 + "+" + "-"*23 + "+" + '-'*8 + '+' + "-"*15 + "+" + "-"*12 + '+' + '-'*12 + '+')
        print("| " + 'Name' + ' '*27 + "| " + 'City' + ' '*14 + "| " + 'Country' + ' '*15 + "| " + 'IATA'+" "*3 +"|   "\
            +"connections" +' ' +"|   " + "inbound" + "  " + "|   outbound" + " "+"|" )
        print("+" + "-"*32 +"+" + "-"*19 + "+" + "-"*23 + "+" + '-'*8 + '+' + "-"*15 + "+" + "-"*12 + '+' + '-'*12 + '+')
        i = lt.size(aeropuertos) 
        while i > lt.size(aeropuertos) - 6: 
            elem = lt.getElement(aeropuertos,i)
            printAirportInfo(elem)
            i -= 1

    elif int(inputs[0]) == 4:
        airport1 = input("Ingrese el codigo IATA del aeropuerto 1: ")
        airport2 = input("Ingrese el codigo IATA del aeropuerto 2: ")
        print("Encontrando componentes fuertemente conectados ...")
        clusters = controller.findClusters(analyzer,airport1,airport2)
        numeroClusters = clusters[1]
        print(f"Se encontraron {numeroClusters} clusters dentro de la red.\n")
        if clusters[0] : 
            print(f"Los aeropuertos {airport1} y {airport2} estan en un mismo cluster")
        else : 
            print("Los aeropuertos no estan conectados")      

    elif int(inputs[0]) == 5:
        ciudad_1  = input("Ingrese el nombre de la ciudad de partida: ")
        ciudad_pais1 = displayCities(ciudad_1) 
        ciudad_2 = input('Ingrese el nombre de la ciudad de llegada: ')
        ciudad_pais2 = displayCities(ciudad_2)
        aeropuerto1 = controller.findNearestAirport(analyzer,ciudad_pais1)
        aeropuerto2 = controller.findNearestAirport(analyzer,ciudad_pais2)
        route = controller.findShortestRoute(analyzer,aeropuerto1['IATA'],aeropuerto2['IATA'])
        for paso in lt.iterator(route) : 
            print("+" + "-"*14 +"+" + "-"*14 + "+" + "-"*16 + "+")
            print("|" + " Departure" + " "*4 + "|" + " Destination" + " "*2 + "|" + " Distance" + " "*7 + "|")
            printRoute(paso)

    elif int(inputs[0]) == 6:
        millas = input("Ingrese la cantidad de millas disponibles del viajero: ")
        mst = controller.mst(analyzer['routes_2'])
        distanciaMillas = controller.distancia(analyzer['routes_2'],mst)
        distanciaKm = (distanciaMillas*1.60)
        faltanExcedenMillas = float(millas) - float(distanciaMillas)
        faltanExcedenKm = ((float(millas)*1.60) - distanciaKm)
        print("La cantidad de nodos de la red de expansion minima es: " ,str(mp.size(mst['mst'])))
        print("La distancia total de la red de expansion minima es de: ",distanciaKm,"km")
        print("+" + "-"*14 +"+" + "-"*14 + "+" + "-"*16 + "+")
        print("|" + " Departure" + " "*4 + "|" + " Destination" + " "*2 + "|" + " Distance" + " "*7 + "|")
        print("+" + "-"*14 +"+" + "-"*14 + "+" + "-"*16 + "+")
        i = 0
        while i < lt.size(mst["mst"]): 
            elem = lt.getElement(mst["mst"],i)
            printArbolExpansion(elem)
            i += 1
        if faltanExcedenMillas < 0:
            print("Al viajero le faltan: " + str(faltanExcedenMillas).strip('-') + " millas.")
            print("Al viajero le faltan: " + str(faltanExcedenKm).strip('-') + " Km.")
        elif faltanExcedenMillas > 0:
            print("Al viajero le sobran: " + str(faltanExcedenMillas) + " millas.")
            print("Al viajero le sobran: " + str(faltanExcedenKm) + " Km.")
        else: 
            print("El viajero usó todas sus millas.")
            print("El viajero usó todas sus Km.")

    elif int(inputs[0]) == 7:
        codigoIATA=input("Ingrese el código IATA del aeropuerto fuera de funcionamiento: ")
        city = controller.findCity(analyzer,codigoIATA)
        listAirports = controller.findNearestAirport(analyzer,city)
        afected = controller.afected(analyzer,listAirports)
        print("La cantidad de paises afectados es: ",lt.size(afected))
        print("La lista de paises afectados es la siguiente: ")
        for city in lt.iterator(afected):
            print(city['IATA'] + city['Name'])

    else:
        sys.exit(0)
sys.exit(0)
