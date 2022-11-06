const WIDTH = 10;
const HEIGHT = 10;
let ALEATORY = true;
let BLOCK = 5;
const STEPCOST = 10;
const pathui = $("#path")[0];
// 0 BLOCKED 1 FREE 2 INITIAL 3 GOAL
let MAP2 =    [3,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,1,
                1,1,1,1,2];
let MAP =   [3,0,1,1,1,1,1,1,1,1,
    1,0,0,0,0,0,0,0,0,0,
    1,1,1,1,1,1,1,1,1,1,
    0,0,0,0,0,0,0,0,0,1,
    1,1,1,1,1,1,1,0,0,1,
    1,1,1,0,1,1,1,1,1,1,
    1,0,1,1,1,1,1,1,1,1,
    1,1,1,0,1,1,1,1,1,1,
    0,1,0,0,0,0,0,1,0,0,
    1,1,1,1,1,1,1,1,1,2];
let path = [];
let actual = 0;
let initial = 0;
let final = 0;
let blocked = [];

analizeInitialMap();
createMapUI();

function runAlgorithm(){
    // Tiene que estar el MAP, actual, initial, final inicializado
    path = run();
    console.log(path)
    if(path!=null){
        showPath();
    }else{
        console.log("ayyy")
    }
    
}

function aleatority(){

    const probability = [];
    for(let i=1;i<=(BLOCK); i++){
        probability.push(i);
    }
    if(probability.includes(Math.floor((Math.random() * 100) + 1))){
        return false;
    }
    return true;
}

function analizeInitialMap(){
    blocked = [];
    if(ALEATORY){
        for(i=0; i<WIDTH*HEIGHT; i++){
            if(aleatority()){
                MAP[i]=1;
            }else{
                MAP[i]=0;
            }
        }
        for(i=0; i<MAP.length; i++){
            if(MAP[i]==1){
                MAP[i]=2;
                break;
            }
        }
        for(i=MAP.length-1; i>=0; i--){
            if(MAP[i]==1){
                MAP[i]=3;
                break;
            }
        }
        
    }
    for(let i=0; i<MAP.length; i++){
        if(MAP[i]==0){
            blocked.push(i+1);
        }
        if(MAP[i]==2){
            initial = i+1;
            actual = initial;
        }
        if(MAP[i]==3){
            final = i+1;
        }
    }
}

function createMapUI(){
    pathui.innerHTML = "";
    for(let i=0; i<HEIGHT; i++){
        const pathRow = document.createElement('div');
        pathRow.classList.add('pathrow');
        pathRow.classList.add('d-flex');
        for(let j=0; j<WIDTH; j++){
            const cell = document.createElement('button');
            cell.onclick = changeCell;
            cell.classList.add('cell');
            cell.id = (i*WIDTH)+j+1;
            if(cell.id == initial){
                cell.classList.add('initial');
            }
            if(cell.id == final){
                cell.classList.add('goal');
            }
            if(blocked.includes(parseInt(cell.id))){
                cell.classList.add('blocked');
            }
            pathRow.appendChild(cell);
        }
        pathui.appendChild(pathRow);
    }
}

function probabilityChanged(){
    ALEATORY = true;
    $('#rangeval').html($("#blockProbability").val());
    BLOCK = $("#blockProbability").val();
    console.log(BLOCK);
    analizeInitialMap();
    createMapUI();
}

function changeCell(){
    const query = "#" + this.id;
    const cell = $(query)[0];
    ALEATORY = false;
    if(cell.classList.contains("blocked")){
        cell.classList.remove("blocked");
        MAP[this.id-1]=1;
    }else{
        cell.classList.add("blocked");
        MAP[this.id-1]=0;
    }
    console.log(cell.classList);
    analizeInitialMap();
}

function showPath(){
    for (var i=0;i<path.length;i++) {
        (function(ind) {
            setTimeout(function(){
                positionateIn(ind);
            }, 1000 + (100 * ind));
        })(i);
    }
    
}

function positionateIn(ind){
    const id = path[ind];
    const query = "#" + id;
    const cell = $(query)[0];
    cell.classList.add("position");
    if(ind-1>=0){
        const query2 = "#" + path[ind-1];
        const acell = $(query2)[0];
        acell.classList.remove("position");
        acell.classList.add("position2");
    }
    if(ind-2>=0){
        const query2 = "#" + path[ind-2];
        const acell = $(query2)[0];
        acell.classList.remove("position2");
    }
}

// ASTAR METHODS

function run(){
    // The map consists in an array of arrays
    const map = this.createMap();
    // Initialize relevant vars
    let actualPosition = actual;
    let initialPosition = initial;
    let finalPosition = final;

    console.log(actualPosition)

    const openList = [];
    const closedList = [];
    let cellNeighbors = [];
    // Validate vars
    if(initialPosition==undefined || finalPosition==undefined){
        return null;
    }

    // Run Algorithm
    while(true){
        // Get neighbors
        cellNeighbors = this.getCellNeighbors(map, actualPosition);
        // Remove neighbors dont reacheable and in the closed list
        cellNeighbors = cellNeighbors.filter(id => !closedList.includes(id) && this.getCellFromMap(map,id).reacheable);
        // Remove neighbors who are in open list and dont have better g
        const actualCell = this.getCellFromMap(map, actualPosition);
        for(let i=0;i<cellNeighbors.length;i++){
            if(actualCell.g+STEPCOST>=this.getCellFromMap(map, cellNeighbors[i]).g && openList.includes(cellNeighbors[i])){
                cellNeighbors.splice(i, 1);
            }
        }
        if(cellNeighbors.length == 0 && openList.length == 0){
            return -1;
        }
        // We have to calculate the heuristics of cellNeighbors
        this.calculateHeuristic(map, cellNeighbors, actualPosition, finalPosition,STEPCOST);
        // Introduce neighbors in open list if they are not already there
        cellNeighbors = cellNeighbors.filter(id => !openList.includes(id));
        openList.splice.apply(openList, [2, 0].concat(cellNeighbors));
        // Choose lower f in openList
        let lower = openList[0];
        for(let i=0; i<openList.length; i++){
            if(this.getCellFromMap(map, openList[i]).f < this.getCellFromMap(map, lower)){
                lower = openList[i];
            }
        }
        // We remove it from open list, add it to closed list and jump to the cell
        openList.splice(openList.indexOf(lower),1);
        closedList.push(lower);
        actualPosition = lower;

        // Test if we have reached goal
        if(actualPosition == finalPosition){
            const steps = [];
            steps.push(finalPosition)
            while(actualPosition!=initialPosition){
                steps.push(this.getCellFromMap(map,actualPosition).parent);
                actualPosition = this.getCellFromMap(map, actualPosition).parent;
            }
            return steps.reverse();
        }
    }
}

function createMap(){
    let id = 0;
    const map = [];
    for(let i=0; i<HEIGHT; i++){
        const row = [];
        for(let j=0; j<WIDTH; j++){
            const cell = {
                id: ++id,
                position: [i, j],
                parent: 0,
                g: 0,
                h: 0,
                f: 0,
                reacheable: this.isReacheable(id)
            }
            row.push(cell);
        }
        map.push(row);
    }
    return map;
}

function isReacheable(id){
    if(MAP[id-1]>0){
        return true;
    }
    return false;
}

function calculateHeuristic(map, ids, actualPosition, finalPosition, stepCost){
    const finalCell = this.getCellFromMap(map, finalPosition);
    const actualCell = this.getCellFromMap(map, actualPosition);
    for(let x=0; x<ids.length; x++){
        const cell = this.getCellFromMap(map,ids[x]);
        // Populate parent
        cell.parent = actualPosition;
        // Calculate g
        cell.g = actualCell.g + stepCost;
        // Calculate h
        cell.h = 0;
        let row = cell.position[0];
        let goalRow = finalCell.position[0];
        while(row != goalRow){
            if(row>goalRow){
                row--;
                cell.h = cell.h +10;
            }
            if(row<goalRow){
                row++;
                cell.h = cell.h+10;
            }
        }
        let column = cell.position[1];
        let goalColumn = finalCell.position[1];
        while(column != goalColumn){
            if(column>goalColumn){
                column--;
                cell.h = cell.h +10;
            }
            if(column<goalColumn){
                column++;
                cell.h = cell.h+10;
            }
        }
        // Populate f
        cell.f = cell.g + cell.h;
        
    }
}

function getCellFromMap(map,Id){
    for(let i = 0; i<map.length; i++){
        for(let j = 0; j<map[i].length; j++){
            if(map[i][j].id == Id){
                return map[i][j];
            }
        }
    }
    return null;
}

function getCellNeighbors(map, Id){
    const neighbors = [];
    const cell = this.getCellFromMap(map, Id);
    const i = cell.position[0];
    const j = cell.position[1];
    if(i-1>=0){
        neighbors.push(map[i-1][j].id);
    }
    if(i+1<map.length){
        neighbors.push(map[i+1][j].id);
    }
    if(j+1<map[i].length){
        neighbors.push(map[i][j+1].id);
    }
    if(j-1>=0){
        neighbors.push(map[i][j-1].id)
    }
    return neighbors;
}