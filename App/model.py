"""
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
 *
 * Contribuciones:
 *
 * Dario Correal - Version inicial
 """


import config as cf
from DISClib.ADT.graph import containsVertex, gr
from DISClib.ADT import map as m
from DISClib.ADT import list as lt
from DISClib.ADT import map as mp
from DISClib.ADT import orderedmap as om 
from DISClib.DataStructures import mapentry as me
from DISClib.Algorithms.Sorting import shellsort as sa
from DISClib.Algorithms.Graphs import scc
from DISClib.Algorithms.Graphs import dijsktra as djk
from DISClib.Utils import error as error
assert cf


# Construccion de modelos

def newAnalyzer():
    try:
        analyzer = {'airports': None,
                    'routes': None,
                    'components': None,
                    'paths': None
                    }
        

        analyzer['airports'] = m.newMap(numelements=10000,
                                     maptype='PROBING',
                                     comparefunction=compareAirports)
        analyzer['Cities'] = m.newMap(numelements=10000,
                                     maptype='PROBING',
                                     comparefunction=compareAirports)

        analyzer['routes'] = gr.newGraph(datastructure='ADJ_LIST',
                                              directed=True,
                                              size=14000,
                                              comparefunction=compareRoutes)
        analyzer['airportsLongitudes'] = om.newMap(omaptype='RBT',comparefunction=compareLongitudes)
        analyzer['airportRoutes'] = m.newMap(numelements=10000,
                                    maptype='PROBING',
                                    comparefunction=compareAirports)

        analyzer['routes_2'] = gr.newGraph(datastructure='ADJ_LIST',
                                              directed=False,
                                              size=14000,
                                              comparefunction=compareRoutes)
        
        analyzer['Cities'] = m.newMap(numelements=10000,
                                    maptype='PROBING',
                                    comparefunction=compareAirports)
        analyzer['city'] = gr.newGraph(datastructure='ADJ_LIST',
                                              directed=False,
                                              size=14000,
                                              comparefunction=compareRoutes)
        
        return analyzer
    except Exception as exp:
        error.reraise(exp, 'model:newAnalyzer')

# Funciones para agregar informacion al grafo
def addAirportbyCode(analyzer,airport) : 
    codes = analyzer['airports']
    code = airport['IATA']
    mp.put(codes,code,airport)

def addCity(analyzer,city):
    cities = analyzer['Cities']
    cityID = city['city_ascii'] + city['country']
    mp.put(cities,cityID,city)

def addRoute(analyzer,route):

    try: 
        airportRoutes = analyzer['airportRoutes']
        distance = float(route['distance_km'])
        existed = mp.contains(airportRoutes,route['Departure'])
        if existed :
            entry = mp.get(airportRoutes,route['Departure'])
            deptRoutes = me.getValue(entry)
            presente = lt.isPresent(deptRoutes['Destinations'],route['Destination'])
        if not existed : 
            deptRoutes = newDeptRoute(route)
            mp.put(airportRoutes,route['Departure'],deptRoutes)
            presente = False
        if not presente :
            lt.addLast(deptRoutes['Destinations'],route['Destination'])
        containsAirport_1 = gr.containsVertex(analyzer['routes'],route['Departure'])
        containsAirport_2 = gr.containsVertex(analyzer['routes'],route['Destination'])
        if not containsAirport_1 : 
            addAirport(analyzer,route['Departure'])
        if not containsAirport_2: 
            addAirport(analyzer,route['Destination'])
        addConnection(analyzer,route['Departure'],route['Destination'],distance)
        
        return analyzer
    except Exception as exp:
        error.reraise(exp, 'model:addStopConnection')

def newDeptRoute(route) : 
    """
    Aniade al indice de airportRoutes un aeropuerto con su lista de rutas 
    """
    airportRoute = {'airport':route['Departure'],'Destinations':''}
    airportRoute['Destinations'] = lt.newList('ARRAY_LIST',compareAirports_2)
    return airportRoute

def addCity_2(analyzer,route) : #TODO: MIRAR SI FUNCIONA
    try: 
        departure = route['Departure']
        infoDeptAirport= mp.get(analyzer['airports'],departure)
        departureAirportCity = infoDeptAirport['City'] + infoDeptAirport['Country']
        destination = route['Destination']
        infoDestAirport = m.get(analyzer['airports'],destination)
    except Exception as exp:
        error.reraise(exp, 'model:addStopConnection')
    
def addAirportbyLongitude(analyzer,airport) : #TODO:terminar arbol RBT de carga de aeropuertos por lat y long. 
    longitudes = analyzer['airportsLongitudes']
    longitude = round(float(airport['Longitude']),2)
    entry = om.get(longitudes,longitude)
    if entry is None : 
        dataEntry = newDataEntry(airport)
        om.put(longitudes,longitude,dataEntry)
    else : 
        dataEntry = me.getValue(entry)
        addLongitudeIndex(dataEntry,airport)
    return  analyzer
    
def newDataEntry(airport) :
    entry = {"lstAirports":''}
    entry['lstAirports'] = lt.newList('ARRAY_LIST')
    return entry

def addLongitudeIndex(dataEntry,Airport) : 
    lst = dataEntry['lstAirports']
    lt.addLast(lst,Airport)
    
def addRoute_2(analyzer):
    try:
        routes = analyzer['routes']
        airportRoutes = analyzer['airportRoutes']
        vertices = gr.vertices(routes)
        for vertice in lt.iterator(vertices) :
            verticeEntry = mp.get(airportRoutes,vertice)
            if verticeEntry != None:
                verticeDestinations = me.getValue(verticeEntry)
                for destination in lt.iterator(verticeDestinations['Destinations']): 
                    if destination != None:
                        destinationEntry = mp.get(airportRoutes,destination)
                        if destinationEntry != None:
                            destinations = me.getValue(destinationEntry)
                            if lt.isPresent(destinations['Destinations'],vertice) : 
                                arco = gr.getEdge(analyzer['routes'],vertice,destination) 
                                costo = arco['weight']
                                containsAirport_1 = gr.containsVertex(analyzer['routes_2'],vertice)
                                containsAirport_2 = gr.containsVertex(analyzer['routes_2'],destination)
                                if not containsAirport_1 :
                                    addAirport_2(analyzer,vertice)
                                if not containsAirport_2 :
                                    addAirport_2(analyzer,destination)    
                                addConnection_2(analyzer,vertice,destination,costo)


        return analyzer
    except Exception as exp:
        error.reraise(exp, 'model:addStopConnection')
    
def addAirport(analyzer,ID):
    try:
        if not gr.containsVertex(analyzer['routes'], ID):
            gr.insertVertex(analyzer['routes'], ID)
        return analyzer
    except Exception as exp:
        error.reraise(exp, 'model:addstop')

def addAirport_2(analyzer,ID):
    try:
        gr.insertVertex(analyzer['routes_2'], ID)
        return analyzer
    except Exception as exp:
        error.reraise(exp, 'model:addAirport')

def addRouteStop(analyzer, service):
    """
    Agrega a una estacion, una ruta que es servida en ese paradero
    """
    entry = m.get(analyzer['airports'], service[''])
    if entry is None:
        lstroutes = lt.newList('SINGLE_LINKED')
        lt.addLast(lstroutes, service['ServiceNo'])
        m.put(analyzer['stops'], service['BusStopCode'], lstroutes)
    else:
        lstroutes = entry['value']
        info = service['ServiceNo']
        if not lt.isPresent(lstroutes, info):
            lt.addLast(lstroutes, info)
    return analyzer


def addRouteConnections(analyzer):
    """
    Por cada vertice (cada estacion) se recorre la lista
    de rutas servidas en dicha estación y se crean
    arcos entre ellas para representar el cambio de ruta
    que se puede realizar en una estación.
    """
    lststops = m.keySet(analyzer['stops'])
    for key in lt.iterator(lststops):
        lstroutes = m.get(analyzer['stops'], key)['value']
        prevrout = None
        for route in lt.iterator(lstroutes):
            route = key + '-' + route
            if prevrout is not None:
                addConnection(analyzer, prevrout, route, 0)
                addConnection(analyzer, route, prevrout, 0)
            prevrout = route


def addConnection(analyzer, origin, destination, distance):
    """
    Adiciona un arco entre dos estaciones
    """
    edge = gr.getEdge(analyzer['routes'], origin, destination)
    if edge is None:
        gr.addEdge(analyzer['routes'], origin, destination, distance)
    return analyzer

def addConnection_2(analyzer, origin, destination, distance):
    """
    Adiciona un arco entre dos estaciones
    """
    edge = gr.getEdge(analyzer['routes_2'], origin, destination)
    if edge is None:
        gr.addEdge(analyzer['routes_2'], origin, destination, distance)
    return analyzer

# Funciones para creacion de datos
def newAirport(ID,aiportInfo) : 
    airport = {"ID":ID,
            "Info":aiportInfo}
    return airport
# Funciones de consulta
def totalAirports(routes) : 
    return gr.numVertices(routes)

def mapSize(map):
    return mp.size(map)
# Funciones utilizadas para comparar elementos dentro de una lista

# Funciones de ordenamiento

# Funciones de comparación
def compareAirports(arpt1,arpt2):
    arptcode = arpt2['key']
    if (arpt1 == arptcode):
        return 0
    elif (arpt1 > arptcode):
        return 1
    else:
        return -1
def compareCities(stop,keyvaluestop):
    stopcode = keyvaluestop['key']
    if (stop == stopcode):
        return 0
    elif (stop > stopcode):
        return 1
    else:
        return -1

def compareRoutes(stop, keyvaluestop):
    stopcode = keyvaluestop['key']
    if (stop == stopcode):
        return 0
    elif (stop > stopcode):
        return 1
    else:
        return -1

def compareLongitudes(longitude1, longitude2):
    if (longitude1 == longitude2):
        return 0
    elif (longitude1 > longitude2):
        return 1
    else:
        return -1
def compareAirports_2(airp1,airp2):
    return airp1 < airp2