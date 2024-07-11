/**
 * @file puzzle.cpp
 * Implementation of puzzle class.
 */
#include "puzzle.h"

void PuzzleState::printState() {
    for (int i = 0; i < 16; i++) {
        std::cout << (int)state_[i] << " ";
        if (i % 4 == 3) {
            std::cout << std::endl; 
        }
    }
    std::cout << std::endl;
}

PuzzleState::PuzzleState() {
    state_ = {1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 0};
    spaceIDX = 15;
}

PuzzleState::PuzzleState(const std::array<uint8_t, 16> state) {
    for (size_t i = 0; i < state.size(); i++) {
        state_[i] = state[i];
        if (state[i] == 0) {
            spaceIDX = i;
        }
    }
}

std::array<uint8_t, 16> PuzzleState::asArray() const {
    std::array<uint8_t, 16> ret;
    for (size_t i = 0; i < state_.size(); i++) {
        ret[i] = state_[i];
    }
    return ret;
}

bool PuzzleState::operator==(const PuzzleState &rhs) const {
    for (size_t i = 0; i < state_.size(); i++) {
        if (state_[i] != rhs.state_[i]) {
            return false;
        }
    }
    return true;
}

bool PuzzleState::operator!=(const PuzzleState &rhs) const {
    for (size_t i = 0; i < state_.size(); i++) {
        if (state_[i] != rhs.state_[i]) {
            return true;
        }
    }
    return false;
}

bool PuzzleState::operator<(const PuzzleState &rhs) const {
    for (int i = 0; i < 16; i++) {
        if (state_[i] < rhs.state_[i]) {
            return true;
        } else if (state_[i] > rhs.state_[i]) {
            return false;
        }
    }

    return false;
}

PuzzleState PuzzleState::getNeighbor(Direction direction) const {
    PuzzleState newState = PuzzleState(state_);

    std::array<uint8_t, 16> zeroArr = {0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0,};

        if (direction == Direction::UP){

            if (spaceIDX > 11) {
                return PuzzleState(zeroArr);
            }
            std::swap(newState.state_[spaceIDX], newState.state_[spaceIDX+4]);
            newState.spaceIDX += 4;

        } else if (direction == Direction::DOWN) {

            if (spaceIDX < 4) {
                return PuzzleState(zeroArr);
            }
            std::swap(newState.state_[spaceIDX], newState.state_[spaceIDX-4]);
            newState.spaceIDX -= 4;

        } else if (direction == Direction::RIGHT) {

            if (spaceIDX % 4 == 0) {
                return PuzzleState(zeroArr);
            }
            std::swap(newState.state_[spaceIDX], newState.state_[spaceIDX-1]);
            newState.spaceIDX -= 1;
        } else if (direction == Direction::LEFT) {

            if (spaceIDX % 4 == 3) {
                return PuzzleState(zeroArr);
            }
            std::swap(newState.state_[spaceIDX], newState.state_[spaceIDX+1]);
            newState.spaceIDX += 1;
        }
    return newState;
}

std::vector<PuzzleState> PuzzleState::getNeighbors() const {
    std::vector<PuzzleState> states;
    std::array<uint8_t, 16> zeroArr = {0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0,};
    PuzzleState zeroState = PuzzleState(zeroArr);

    PuzzleState psUP = getNeighbor(Direction::UP);
    if (psUP != zeroState) {
        states.push_back(psUP);
    }

    PuzzleState psRIGHT = getNeighbor(Direction::RIGHT);
    if (psRIGHT != zeroState) {
        states.push_back(psRIGHT);
    }

    PuzzleState psDOWN = getNeighbor(Direction::DOWN);
    if (psDOWN != zeroState) {
        states.push_back(psDOWN);
    }

    PuzzleState psLEFT = getNeighbor(Direction::LEFT);
    if (psLEFT != zeroState) {
        states.push_back(psLEFT);
    }

    return states;
}

int PuzzleState::manhattanDistance(const PuzzleState desiredState) const {
    int sum = 0;
    for (size_t i = 0; i < state_.size(); i++) {
        int curr = (int)state_[i];
        if (curr != 0) {
            int curr_x = i/4;
            int curr_y = i%4;

            int desired_x;
            int desired_y;
            for (size_t j = 0; j < desiredState.state_.size(); j++) {
                if (desiredState.state_[j] == curr) {
                    desired_x = j/4;
                    desired_y = j%4;
                }
            }

            sum += std::abs(curr_x - desired_x) + std::abs(curr_y - desired_y);
        }
    }
    return sum;
}

std::vector<PuzzleState> solveAstar(const PuzzleState& startState, const PuzzleState &desiredState, size_t *iterations) {
    std::vector<PuzzleState> solution;
    std::map<PuzzleState, PuzzleState> prev;
    std::map<PuzzleState, int> cost;
    std::map<PuzzleState, int> visited;

    std::priority_queue<std::pair<int, PuzzleState>, std::vector<std::pair<int, PuzzleState>>, std::greater<std::pair<int, PuzzleState>>> q;

    (*iterations) = 0;

    q.push({startState.manhattanDistance(desiredState), startState});
    visited[startState] = 1;
    cost[startState] = 0;

    while (!q.empty()) {
        PuzzleState currState = q.top().second;
        int currCost = cost[currState];
        q.pop();

        (*iterations)++;

        if (currState == desiredState) {
            //do stuff
            while (currState != startState) {
                solution.push_back(currState);
                currState = prev[currState];
            }
            solution.push_back(startState);
            std::reverse(solution.begin(), solution.end());
            return solution;
        }

        std::vector<PuzzleState> neighbors = currState.getNeighbors();

        currCost++;
        for (unsigned i = 0; i < neighbors.size(); i++) {
            if (visited.find(neighbors[i]) == visited.end() || currCost < cost[neighbors[i]]) {
                q.push({neighbors[i].manhattanDistance(desiredState) + currCost, neighbors[i]});
                cost[neighbors[i]] = currCost;
                visited[neighbors[i]] = 1;
                prev[neighbors[i]] = currState;
            }
        }
    }

    return std::vector<PuzzleState>();
}

std::vector<PuzzleState> solveBFS(const PuzzleState& startState, const PuzzleState &desiredState, size_t *iterations) {
    // solution that keeps track of where it came from
    std::vector<PuzzleState> solution;
    // maps next puzzlestate to curr
    std::map<PuzzleState, PuzzleState> prev;
    //visited map, used map for efficiency c ouldn't figure out how to implement unordered_set
    std::map<PuzzleState, int> visited;


    std::queue<PuzzleState> q;
    //itialize iterations to 0 so that it is not pointing in random memory
    (*iterations) = 0;

    q.push(startState);
    visited[startState] = 1;

    while (!q.empty()) {
        PuzzleState curr = q.front();
        q.pop();

        (*iterations)++;
        // if we are where we wannna be retrace the steps
        if (curr == desiredState) {
            while (curr != startState) {
                solution.push_back(curr);
                curr = prev[curr];
            }
            solution.push_back(startState);
            std::reverse(solution.begin(), solution.end());
            return solution;
        }
        // grab neighbors to be checked out
        std::vector<PuzzleState> neighbors = curr.getNeighbors();
        // for each neighbor add to queue if it hasnt been visited also mark as visited and map the prev
        for (size_t i = 0; i < neighbors.size(); i++) {
            if (visited.find(neighbors[i]) == visited.end()) {
                q.push(neighbors[i]);
                prev[neighbors[i]] = curr;
                visited[neighbors[i]] = 1;
            }
        }
    }

    return std::vector<PuzzleState>();
}

std::vector<PuzzleState> solveBFS(const PuzzleState &startState, const PuzzleState &desiredState) {
    size_t i;
    return solveBFS(startState, desiredState, &i);
}

std::vector<PuzzleState> solveAstar(const PuzzleState &startState, const PuzzleState &desiredState) {
    size_t i;
    return solveAstar(startState, desiredState, &i);
}
