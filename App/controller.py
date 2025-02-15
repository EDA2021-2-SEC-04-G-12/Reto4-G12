﻿"""
 * Copyright 2020, Departamento de sistemas y Computación,
 * Universidad de Los Andes
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

from DISClib.ADT.graph import adjacents, containsVertex, gr, vertices
from DISClib.ADT import map as m
import config as cf
import model
import csv



"""
El controlador se encarga de mediar entre la vista y el modelo.
"""

# Inicialización del Analizador 

def init():
    analyzer = model.newAnalyzer()
    return analyzer


# Funciones para la carga de datos

def loadServices(analyzer):
    routesfile = cf.data_dir + "routes-utf8-small.csv"
    input_file1 = csv.DictReader(open(routesfile, encoding="utf-8"),
                                delimiter=",")
    lastroute = None
    airportsfile = cf.data_dir +  "airports-utf8-small.csv"
    input_file2 = csv.DictReader(open(airportsfile, encoding="utf-8"),
                                delimiter=",")
    citiesfile = cf.data_dir + "worldcities-utf8.csv"
    input_file3 = csv.DictReader(open(citiesfile, encoding="utf-8"),
                                delimiter=",")
    i = 1
    airport_inicial = None
    airport_final = None
    print("Cargando aeropuertos ...")
    for airport in input_file2 : 
        model.addAirportbyCode(analyzer,airport)
        model.addAirportbyLongitude(analyzer,airport)
        model.addAirporttoGraph(analyzer,airport)
        if i == 1:
            airport_inicial = airport 
        if i == gr.numVertices(analyzer['routes']):
            airport_final = airport
    j = 0 
    print('Cargando rutas ...')
    for route in input_file1:
        model.addRoute(analyzer,route)
        j += 1 
    #routes_2 = model.addRoute_2(analyzer)
    routes_3 = model.addRoute_3(analyzer)
    k = 1
    airport_inicial_1 = None
    airport_final_1 = None
    print('Cargando ciudades ...')
    for city in input_file3 : 
        model.addCity(analyzer,city)
        if i == 1:
            airport_inicial_1 = city 
        if i == m.size(analyzer['city-country']):
            airport_final_1 = city
    
    airports_1 = model.totalAirports(analyzer['routes'])
    routes_1 = model.totalRoutes(analyzer['routes'])
    airports_2 = model.totalAirports(analyzer['routes_2'])
    routes_2 = model.totalRoutes(analyzer['routes_2'])
    cities = model.mapSize(analyzer['city-country'])
    
    return analyzer,airports_1,routes_1,airports_2,routes_2,cities,airport_inicial, airport_final,airport_inicial_1,airport_final_1

# Funciones de ordenamiento

# Funciones de consulta sobre el catálogo

def cityMap(analyzer,city):
    return model.cityMap(analyzer,city)
def findNearestAirport(analyzer,ciudad_pais1): 
    return model.findNearestAirport(analyzer,ciudad_pais1)
def masConectados(analyzer) :
    """
    Retorna al view los aeropuestos mas conectados en la red de rutas
    """
    aeropuertos = model.masConectados(analyzer)
    return aeropuertos

def findClusters(analyzer,airport1,airport2): 
    clusters = model.findClusters(analyzer,airport1,airport2)
    return clusters

def findShortestRoute(analyzer,ciudad_1,ciudad_2) : 
    esoSi = model.findShortestRoute(analyzer,ciudad_1,ciudad_2)
    return esoSi

def mst(grafo):
    return model.mst(grafo)

def distancia(grafo,mst):
    return model.distancia(grafo,mst)

def nearAirports(analyzer,codeIATA):
    return model.nearAirports(analyzer,codeIATA)

def afected(analyzer,listAirports):
    return model.afected(analyzer,listAirports)

def findCity(analyzer,codeIATA):
    return model.findCity(analyzer,codeIATA)