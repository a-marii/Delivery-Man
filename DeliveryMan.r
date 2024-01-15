Manhattan_Distance = function(x1, y1, x2, y2) {
  dist = abs(x1 - x2) + abs(y1 - y2)
  return (dist)
}

FindClosestPackage = function(carInfo, packageMatrix) {
  unpicked = which(packageMatrix[, 5] == 0)
  if (length(unpicked) == 0) {
    return(0)
  }
  else {
    nearestPackage = unpicked[1]
    minDistance = Manhattan_Distance(carInfo$x, carInfo$y, packageMatrix[nearestPackage, 1], packageMatrix[nearestPackage, 2])
    if (length(unpicked) > 1) {
      for (i in 2:length(unpicked)) {
        dist = Manhattan_Distance(carInfo$x, carInfo$y, packageMatrix[unpicked[i], 1], packageMatrix[unpicked[i], 2])
        if (dist < minDistance) {
          minDistance = dist
          nearestPackage = unpicked[i]
        }
      }
    }
    return (nearestPackage)
  }
}


initializeNodes <- function(carx, cary)
{
  rows <- list()
  for (y in 1:10)
  {
    row = list()
    for (x in 1:10)
    {
      row[[x]] <-
        list(
          x = x,
          y = y,
          f = 0,
          g = 0,
          h = Manhattan_Distance(x, y, carx, cary),
          parent = list(x = 0, y = 0)
        )
    }
    rows[[y]] <- row
  }
  return (rows)
}

getNeighbors <- function(car, nodes)
{
  neighbors <- list()
  x <- car$x
  y <- car$y
  if (x - 1 >= 1)
    neighbors[[length(neighbors) + 1]] <- nodes[[y]][[x - 1]]
  if (x + 1 <= 10)
    neighbors[[length(neighbors) + 1]] <- nodes[[y]][[x + 1]]
  if (y - 1 >= 1)
    neighbors[[length(neighbors) + 1]] <- nodes[[y - 1]][[x]]
  if (y + 1 <= 10)
    neighbors[[length(neighbors) + 1]] <- nodes[[y + 1]][[x]]
  return (neighbors)
}


calculateNeighborsCost <- function(node, nodes, roads)
{
  x = node$x
  y = node$y
  hroads = roads$hroads
  vroads = roads$vroads
  neighbors = getNeighbors(node, nodes)
  
  for (neighbor in neighbors) {
    dx <- neighbor$x - x
    dy <- neighbor$y - y
    # Move horizontally so get the horizontal cost
    if (x + 1 == neighbor$x)
    {
      neighbor$g <- node$g + hroads[x, y]
    }
    else if (x - 1 == neighbor$x)
    {
      neighbor$g <- node$g + hroads[x - 1, y]
    }
    # Move vertically so get the vertical cost
    else if (y + 1 == neighbor$y)
    {
      neighbor$g <- node$g + vroads[x, y]
    }
    else if (y - 1 == neighbor$y)
    {
      neighbor$g <- node$g + vroads[x, y - 1]
    }
    neighbor$f <- neighbor$g + neighbor$h
    nodes[[neighbor$y]][[neighbor$x]] <- neighbor
  }
  return (getNeighbors(node, nodes))
}

getBestNode <- function(frontier, goal)
{
  bestIndex <- NULL
  bestNode <- NULL
  bestF <- Inf
  for (i in 1:length(frontier))
  {
    node = frontier[[i]]
    if (length(node$f) == 0)
      node$f = 0
    if (node$f < bestF)
    {
      bestIndex <- i
      bestF <- node$f
      bestNode <- node
    }
  }
  return (bestIndex)
}

SetChecking <- function(set, x, y) {
  if (length(set) == 0) {
    return (0)
  }
  for (i in 1:length(set)) {
    if (x == set[[i]]$x && y == set[[i]]$y) {
      return (i)
    }
  }
  return(0)
}

FindPath <- function(final_node, ClosedList) {
  parent_node = final_node
  run = TRUE
  if (all(final_node$parent == c(x = 0, y = 0))) {
    return (final_node)
  }
  while (run == TRUE) {
    for (index_node in 1:length(ClosedList)) {
      if (all(c(ClosedList[[index_node]]$x, ClosedList[[index_node]]$y) == parent_node$parent)) {
        if (all(ClosedList[[index_node]]$parent == c(0, 0))) {
          run = FALSE
          return(parent_node)
        }
        
        parent_node = ClosedList[[index_node]]
        ClosedList = ClosedList[-index_node]
        break
      }
    }
    
  }
  
}

AstarAlgorithm = function(roads, car, packages,closest_packagex,closest_packagey){
  nodes = initializeNodes(car$x, car$y)
  OpenList = list()
  ClosedList = list()
  current_index = 1
  current_x = car$x
  current_y = car$y
  current_Node <- nodes[[current_y]][[current_x]]
  OpenList[[length(OpenList) + 1]] = nodes[[current_y]][[current_x]]
  while (TRUE) {
    neighbors <- calculateNeighborsCost(current_Node, nodes, roads)
    for (neighbor in neighbors) {
      isinOpenList=SetChecking(OpenList, neighbor$x, neighbor$y)
      if (isinOpenList==0 &&
          SetChecking(ClosedList, neighbor$x, neighbor$y) == 0) {
        neighbor$parent = list(x = current_x, y = current_y)
        OpenList[[length(OpenList) + 1]] = neighbor
      }
      else if(isinOpenList!=0 && OpenList[[isinOpenList]]$f>neighbor$f){
        neighbor$parent = list(x = current_x, y = current_y)
        OpenList[[isinOpenList]] = neighbor
      }
    }
    OpenList = OpenList[-current_index]
    current_index <- getBestNode(OpenList, list(x = current_x, y = current_y))
    ClosedList[[length(ClosedList) + 1]] <- current_Node
    current_x  = OpenList[[current_index]]$x
    current_y = OpenList[[current_index]]$y
    
    current_Node = OpenList[[current_index]]
    if (Manhattan_Distance(current_x, current_y, closest_packagex,closest_packagey) ==
        0) {
      next_node = FindPath(current_Node, ClosedList)
      next_move = nextMove(car, next_node)
      car$nextMove = next_move
      return(car)
    }
  }
}

myFunction <- function(roads, car, packages)
{
  if (car$load == 0) {
    package_index = FindClosestPackage(car, packages)
    if(all(c(car$x, car$y)==c(packages[package_index,1],packages[package_index,2]))){
      car$nextMove=5
      return (car)
    }
    car=AstarAlgorithm(roads, car, packages, packages[package_index,1], packages[package_index,2])
    
  }
  else{
    if(all(c(car$x, car$y)==c(packages[car$load,3], packages[car$load,4]))){
      car$nextMove=5
      return (car)
    }
    car = AstarAlgorithm(roads, car, packages, packages[car$load,3], packages[car$load,4])
  }
  return (car)
  
}
nextMove = function(car, next_node) {
  if (next_node$x == car$x + 1) next_move = 6
  else if (next_node$x == car$x - 1) next_move = 4
  else if (next_node$y == car$y + 1) next_move = 8
  else if (next_node$y == car$y - 1 ) next_move = 2
  else next_move = 5
  return (next_move)
  
}
