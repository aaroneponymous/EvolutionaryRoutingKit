/**
 * @brief
*/

/**
 * @brief: Ant-Colony System Implementation Details
 * 
 *         1. Tour Construction of ACS
 *            - Use the Initial Parameter and Compute Values
 * 
 *            @parameter: 1. k (number of ants)
 * 
 *            - An ant k in node i chooses the next node j with a probability defined
 *              by the "Random Proportional Rule" | P of k moving from i to j:
 *            - If j belongs to N(k)(i) (feasible neighborhood)
 * 
 *            @equation:  
 * 
 *            @parameters: 
 *                        2.  history_coefficient (alpha)   - Controls the influence of the pheromone train 
 *                                                            in decision making
 * 
 *                        3.  heurisitic_coefficient (beta) - Determines the importance of the heuristic
 *                                                            information in the decision making
 * 
 *                        4.  pheromone_matrix              - Matrix to record pheromones deposit 
 *                                                          - T (ij): Amount of pheromones deposited for a state
 *                                                            transition from i to j
 *                                                          - n (ij): Desirability of state transition i to j
 *                                                            (a priori knowledge, typically 1/d(ij) d: distance
 *                                                            from i to j) 
 *                        5.  cost_matrix
 * 
 *              @decision: 
 *                        In ACS the Random Proportional Rule with probability q0 E [0, 1] for the 
 *                        next city visit is defined as:
 *          
 *                        - j =
 * 
 *                        - Otherwise J, where J is a random variable given by EQ (1)
 * 
 *          2. Global Pheromone Trail Update
 * 
 *             - After each iteration, the shortest tour (global-best tour) of this iteration is determined
 *               and arcs belonging to this tour receive "extra" pheromone, so only the global-best tour
 *               allow ants to add pheromone after each iteration by the global updating rule:
 * 
 *             - @function: T(ij) (t + 1) = ( 1 - rho)Tij(t) + rho delta T(global)(ij)[t] with all (i,j) E global-best tour
 * 
 * 
 * 
 *             - @note: Pheromone Trail Update Rule is only applied to the arcs of the global-best tour,
 *                      not to all arcs like in AS
 * 
 *             @parameters:
 *                      
 *                         1. 
 * 
 * 
 *          3. Local Pheromone Trail Update
 * 
 * 
 *             - In addition the ants use a local update rule that is immediately applied after visiting an arc during the construction
 *               in ACS
 *
 *             - @function: T(ij) = (1 - decay) x Tij + decay x Tij
 *            
*/