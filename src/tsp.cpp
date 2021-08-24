/* Autores:
 * Heitor Tonel Ventura - 2086883
 * José Henrique Ivanchechen - 2090341
 */

#include "quick_hull.cpp"
#include <algorithm>

/* Escolha das estruturas de dados:
 * Para representação do ciclo, mantém-se a lista encadeada retornada do algoritmo de quick_hull, pois
 * é necessário inserir elementos em posição aleatória e listas encadeadas permitem essa operação em O(1).
 * 
 * Para a representação dos pontos a serem analisados, será utilizada uma lista encadeada feita a partir da
 * conversão de uma lista sequencial. Isso foi decidido pois é necessário remover os pontos do ciclo dessa lista
 * auxiliar e como estes poderão estar em posição aleatória, a lista encadeada garante a operação em O(1).
 * 
 */

/* remove_cycle_from_points
 * cycle: std::list<Point2D>&, lista de pontos dentro de um ciclo
 * points: std::list<Point2D>&, lista de pontos que também contém os pontos do ciclo
 * Este método remove os pontos de points que estão tanto em points quanto em cycle.
 * O retorno da função é dado dentro da lista points.
 * 
 * - Análise de complexidade:
 * A complexidade da função é dada pelos laços de repetição dentro dela, o primeiro laço de repetição
 * será executado Ci vezes, onde Ci é o número de pontos dentro do ciclo inicial, ou seja, do fecho convexo.
 * O segundo laço de repetição, que está dentro do primeiro, será executado N vezes, porém como o mesmo
 * está dentro do primeiro laço, ele será executado no máximo N*C vezes. Em média, assumindo que o ponto
 * a ser buscado está no meio da lista, o custo médio será Ci * (N/2), pois o loop interno para a iteração
 * quando encontra o valor buscado. As operações executadas dentro desse loop são de tempo constante.
 * Logo, a complexidade de remove_cycle_from_points é limitada por O(N*Ci).
 * 
 * Melhor caso:
 * Quando Ci é constante, o custo de O(N*Ci) será linear da ordem de O(N). Ci tende a ser bem pequeno nos testes realizados
 * com uma distribuição em espaço fixo e uma distribuição retangular. Com um N suficientemente grande, Ci tende a 4, pois o 
 * fecho convexo da distribuição retangular será o retângulo que forma a bounding box de todos os pontos.
 * 
 * Pior caso:
 * Caso Ci seja da ordem de N. Isso ocorre quando o fecho convexo é muito próximo de um ciclo hamiltoniano e o seu tamanho cresce
 * em conjunto com o tamanho da entrada. Neste caso, o custo do algoritmo será da ordem de O(N^2). Esse caso não é observado nos
 * testes do algoritmo, pois a distribuição retangular de espaço fixo tende a um Ci constante.
 *
 * Caso médio:
 * Assumindo a distribuição de pontos fornecida, percebe-se que Ci tende a um fecho convexo constante, formado pela bounding box
 * do espaço fechado em que os pontos são gerados. Por causa disso, percebe-se que Ci tem um comportamento constante nesses casos
 * de utilização. Por causa disso, a execução média nessa situação também linear e portanto O(N).
 *
 * - Corretude:
 * 
 * Prova por loop invariante:
 * 
 * Invariante de loop: O conjunto S não conterá os pontos de um subvetor de pontos analisados Pa que fazem parte do ciclo C.
 * 
 * Inicialização:
 * |Pa| é zero, e portanto, nenhum ponto foi removido dentro de S.
 * 
 * Pa = []
 * P é o espaço de todos os pontos analisados.
 * S = P
 * 
 * Manutenção:
 * Ao fim da iteração do laço, foi comparado o ponto do ciclo da iteração atual com os pontos dentro
 * de points e removido o ponto da iteração dentro da lista de points. Portanto, se o ponto i analisado
 * estiver em C, este será removido de S, mantendo a invariante.
 * 
 * Pa = [1:i]
 * S* = S da iteração i - 1
 * S = S* - i caso i pertença a C
 * S = S*     caso contrário
 * 
 * Término:
 * Após o final do loop, o subvetor Pa conterá todos os pontos e o conjunto S conterá todos os pontos em P
 * que não estiverem dentro do ciclo analisado.
 * 
 * Pa = [1:i]
 * S = P - C
 * 
 * Portanto, assumindo que A seja a lista de pontos e B seja a lista de pontos dentro do ciclo,
 * a lista de points conterá A - B ao fim da execução.
 */
void remove_cycle_from_points(const std::list<Point2D>& cycle, std::list<Point2D>& points) {
    // Começa iteração pelos pontos dentro do ciclo (convex hull)
    for (const Point2D& cyclePoint : cycle) {
        // Para cada ponto dentro do ciclo, faz um loop por todos os pontos
        for (std::list<Point2D>::iterator point_it = points.begin(); point_it != points.end(); ++point_it) {
            Point2D point = *point_it; // Pega o objeto sendo apontado pelo iterador do loop
            
            // Verifica se o ponto do ciclo é igual ao ponto da iteração
            // Se for, ele é removido de points e o loop interno é quebrado (já que o ponto já foi removido)
            if (point.x == cyclePoint.x && point.y == cyclePoint.y) {
                points.erase(point_it);
                break;
            }
        }
    }
}

/* dist
 * p1: Point2D, primeiro ponto a ser calculado a distância.
 * p2: Point2D, segundo ponto a ser calculado a distância.
 * Função que realiza o cálculo de distância entre dois pontos (p1 e p2).
 * Retorna a distância euclidiana entre os pontos p1 e p2.
 * 
 * - Análise de complexidade:
 * A complexidade da função é dada em tempo constante, já que são realizadas apenas operações
 * aritméticas.
 * Logo, a complexidade de dist é dada por O(1).
 *
 * - Corretude:
 * A função executa uma tradução direta da função matemática de cálculo de distância euclidiana.
 * Pode ser trivialmente provada a partir do teorema de pitágoras.
 * c^2 = a^2 + b^2
 * a = (p1.x - p2.x)
 * b = (p1.y - p2.y)
 * 
 * dist_euclidiana = sqrt(a^2 + b^2)
 * 
 */
float dist(const Point2D& p1, const Point2D& p2) {
    return sqrt(
        (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
    );
}

/* calc_dist
 * cycle: std::list<Point2D>&, ciclo para ser calculado a distância
 * Função que calcula a distância percorrida entre os pontos do ciclo informado.
 * Usado para calcular o tamanho do caminho encontrado pelo algoritmo.
 * Retorna a soma da distância euclidiana entre os pontos.
 * 
 * - Análise de complexidade:
 * O algoritmo de calc_dist começa declarando uma variável, que é de tempo constante, O(1).
 * Após isso, é realizado um loop que será executado para cada ponto dentro do ciclo, logo,
 * o loop será executado N vezes, O(N).
 * Dentro do laço, todas operações realizadas são de tempo constante, O(1).
 * Ao final do algoritmo, é executado uma operação de retorno de tempo constante, O(1).
 * Logo, calc_dist é limitada por tempo linear, O(N).
 *
 * - Corretude:
 * 
 * Prova por loop invariante:
 * 
 * Invariante de loop: A variável total_dist terá a soma da distância euclidiana dos pontos dentro
 * do subvetor dos pontos analisados Pa.
 * 
 * Inicialização:
 * |Pa| é zero, e portanto a soma da distância total será de 0.
 * 
 * Pa = []
 * TD = 0
 * 
 * Manutenção:
 * Ao fim da iteração do laço, será calculado a distância entre o ponto atual (i) e o próximo
 * ponto (j) a ser analisado.
 * 
 * Pa = P[1:i]
 * TD = soma atual da distância dos pontos analisados em Pa
 * 
 * Término:
 * Após o final do loop, a variável total_dist conterá a soma de todas as distâncias do ciclo
 * analisado. 
 * 
 * Pa = P[1:N]
 * TD = soma atual da distância dos pontos dentro do ciclo.
 * 
 * Portanto, a variável retornada ao final do algoritmo será a soma de todas as distâncias do ciclo.
 */
float calc_dist(std::list<Point2D>& cycle) {
    // Soma da distância a ser retornada ao final da função
    float total_dist = 0;
    
    // Para cada ponto i dentro do ciclo...
    for (auto iit = cycle.begin(); iit != cycle.end(); ++iit) {
        auto jit = std::next(iit); // Pega o próximo ponto (depois de i)
        
        if (jit == cycle.end()) { // Se o ponto j não existir...
            jit = cycle.begin(); // Ponto j será o começo, para fechar o ciclo
        }

        // Aqui o ponto j será o sucessor de i dentro do ciclo

        // Pega o objeto dos pontos apontado por seus iteradores
        Point2D& i = *iit,
                 j = *jit;
        // Soma a distância entre i e j dentro de total_dist
        total_dist += dist(i, j);
    }

    // Retorna a distância final
    return total_dist;
}

/* tsp
 * points: std::list<Point2D>&, lista de pontos da entrada que não estão no ciclo ainda. 
 * cycle: std::list<Point2D>&, fecho convexo previamente calculado.
 * Função que adiciona os pontos da lista points na lista cycle de forma a construir um
 * ciclo hamiltoniano, utilizado para retornar uma solução aproximada do
 * problema do caixeiro viajante (tsp) com a heurística de inserção mais barata.
 * 
 * - Análise de complexidade:
 * 
 * Inicialmente, variáveis são definidas, que têm um custo O(1).
 * Após isso, o algoritmo executa um loop determinado pela condição de haver ainda elementos na lista points. Como em cada
 * iteração é removido um ponto dessa lista, esse loop executará da ordem de O(N - Ci), com Ci sendo o tamanho do fecho
 * convexo. 
 * Após isso, é iterado por todos os pontos da lista points novamente. Porém, nesse caso, serão executadas N - C vezes para
 * cada iteração do loop acima e com C sendo o tamanho do ciclo atual.
 * Dentro desse loop, um terceiro loop aninhado estará presente e executará C vezes em cada iteração do loop externo,
 * com C sendo novamente o tamanho do ciclo atual.
 * Percebe-se que C varia de Ci até N e portanto, o custo do loop mais interno se dará pela expressão:
 * sum com C = Ci até N de ((N - C) * C)
 * = sum com C = Ci até N de (NC - C^2)
 * = sum com C = Ci até N de (NC) - sum com C = Ci até N de (C^2) 
 * Analisando o fator que cresce com o aumento da entrada:
 * sum com C = Ci até N de (NC)
 * = N * (sum com C = Ci até N de (C))
 * = N * (sum com C = 1 até N de (C) - sum com C = 1 até Ci de (C))
 * = N * (N * (N+1) / 2 - Ci * (Ci + 1) / 2)
 * = N * ((N^2)/2 + N/2 - (Ci^2)/2 - Ci/2)
 * = (N^3)/2 + ^2/2 - N * (Ci^2)/2 + N * Ci/2)
 * Analisando o fator que diminui com o aumento da entrada:
 *  - sum com C = Ci até N de (C^2) 
 * = - (sum com C = 1 até N de (C^2) - sum com C = 1 até Ci de (C^2))
 * = - (((N * (N+1) * (2N+1))/6 - (Ci * (Ci+1) * (2Ci+1))/6))
 * = - (((N^2 + N) * (2N + 1))/6 - ((Ci^2 + Ci) * (2Ci + 1))/6))
 * = - ((2N^3)/6 + (2N^2)/6 + (N^2)/6 + N^6 - ((2Ci^3)/6 + (2Ci^2)/6 + (Ci^2)/6 + Ci/6))
 * = - ((2N^3)/6 + (N^2)/2 + N^6 - (2Ci^3)/6 - (Ci^2)/2 - Ci/6)
 * = - (N^3)/3 - (N^2)/2 - N^6 + (Ci^3)/3 + (Ci^2)/2 + Ci/6
 * Percebe-se que no fator que cresce, o termo que domina é o (N^3) / 2 e no fator que reduz o - (N^3)/3.
 * Como resultado final o custo do loop mais interno é limitado por (N^3)/6 e portanto ele executa em ordem de O(N^3).
 * Dentro do loop mais interno, são executadas operações constantes. 
 * Ao fim dos dois loops internos, é feita uma inserção e uma remoção em lista encadeada, com custo constante também.
 * Portanto algortimo é limitado pelo custo do loop mais interno e é da ordem de O(N^3).
 * 
 * - Corretude:
 * Definição:
 *  Um ciclo hamiltoniano é um ciclo que passa por todos os vértices de um grafo somente uma vez.
 *  Todo grafo completo com 2 ou mais elementos possui um ciclo hamiltoniano. Portanto, como o conjunto de pontos
 *  analisados pode ser visto como um grafo completo, sempre pode-se encontrar um ciclo hamiltoniano.
 * 
 * Invariante de loop 1 (loop mais externo):
 *  Dado um grafo completo G(E, V) e um ciclo C:
 *  C sempre será um ciclo hamiltoniano do conjunto de vértices analisados Va que está contido em V.
 * 
 * Inicialização:
 *  C contém os pontos do fecho convexo, que por definição é um ciclo.
 *  Os pontos analisados em Va são aqueles retornados pelo fecho convexo.
 *  
 *  C = a, b, c, ... n, onde C é o caminho que representa o fecho convexo.
 *  Va contém todos os pontos de C
 *  Vr contém os pontos de V - C, que serão atribuídos a C
 * 
 * Manutenção:
 *  Dado uma aresta A(i, j) dentro do ciclo que pertence a E, escolhe-se um k.
 *  A partir disso, é removido A do ciclo e adicionado duas novas arestas, IK e KJ.
 *  
 *  Ao substituir A por IK e KJ, mantém-se o ciclo hamiltoniano em Va, pois os vértices previamente inseridos i e j se mantém no ciclo
 *  e k é adicionado somente uma vez.
 * 
 *  C = a, b, c, ... i, k, j, ... n, onde k é o ponto inserido nessa iteração.
 *  Va contém todos os pontos de C
 *  Vr contém os pontos V - C
 * 
 * Término:
 *  Após o fim do loop externo, todos os vértices em V foram analisados e inseridos em C, logo Vr está vazio.
 * 
 *  C = a, b, c, ... n, onde C contém todos os pontos de V.
 *  Va contém todos os pontos de V
 *  |Vr| = 0
 * 
 * Logo, essa invariante prova que o resultado retornado pela função será um ciclo hamiltoniano.
 * 
 * Invariante de loop 2 (primeiro loop interno):
 *  Dado um grafo completo G(E, V) e um ciclo C:
 *  O vértice P conterá sempre o vértice que minimiza a inserção de P em C para um subvetor Va de pontos analisados.
 *  
 * Inicialização:
 *  |Pa| é zero, e portanto não há vértice a ser analisado.
 *  
 *  Pa = []
 *  P = não existe
 * 
 * Manutenção:
 *  Dado que:
 *  diP é a distância do vértice i até o vértice P, ou o peso da aresta IP.
 *  djP é a distância do vértice j até o vértice P, ou o peso da aresta PJ.
 *  dij é a distância do vértice i até o vértice i, ou o peso da aresta IJ.
 *  Ao fim de uma iteração do laço, é visto que P mantém o vértice que minimiza a expressão D = diP + djP - dij, para
 *  o vetor de pontos analisados Pa.
 *  
 *  Seja X0 o custo de C atual e X o custo de C com a inserção de P, logo
 *  X = X0 + D, pois a aresta IJ é removida e as arestas IP e PJ são adicionadas em seu lugar.
 *  Logo, escolher o ponto que minimiza D implica em escolher o ponto que minimiza a inserção de P em C.
 * 
 *  Pa = [1:i]
 *  P = vértice que minimiza a inserção em C para o Pa analisado.
 * 
 * Término:
 *  Após o término do loop, P conterá o vértice que minimiza D.
 * 
 *  Pa = P[1:N]
 *  P = vértice que terá o minimiza a inserção em C.
 *  
 * Logo, essa invariante prova que o primeiro loop interno sempre seleciona um P que minimiza a inserção em C.
 * 
 * Portanto, prova-se que o algoritmo irá retornar um ciclo hamiltoniano como resultado e que esse ciclo usará uma heurística
 * de minimizar o custo de cada inserção.
 */
void tsp(std::list<Point2D>& points, std::list<Point2D>& cycle) {
    // Variáveis para guardar o ponto com menor distância dentro do ciclo
    float min_dist = INF;
    std::list<Point2D>::iterator min_jit;
    std::list<Point2D>::iterator min_point;

    // Enquanto ainda houver pontos a serem analisados...
    while (points.size() > 0) {
        min_dist = INF;
        // Para cada ponto que ainda não está dentro do ciclo...
        for (std::list<Point2D>::iterator point = points.begin(); point != points.end(); ++point) {

            // Loop para calcular a tripla de pontos que minimiza a operação dik + djk - dij
            for (auto iit = cycle.begin(); iit != cycle.end(); ++iit) {
                auto jit = std::next(iit); // Pega o sucessor do ponto i dentro do ciclo
                
                if (jit == cycle.end()) { // Se o ponto j não existir...
                    jit = cycle.begin(); // O ponto j será o começo do ciclo, para fechar o ciclo
                }
                
                // Aqui o vértice j será o sucessor de i
                // O vértice k será o vértice a ser analisado
                
                // Retorna os vértices i e j apontados por seus iteradores
                Point2D& i = *iit,
                         j = *jit;
                
                // Calcula distância entre i-k, j-k e i-j
                // k é o vértice do loop externo, point
                float dik = dist(i, *point);
                float djk = dist(j, *point);
                float dij = dist(i, j);
                float new_min_dist = dik + djk - dij;

                // Se a distância for menor do que alguma já calculada...
                if (new_min_dist < min_dist) {
                    // Substitui com as novas informações para realizar a inserção dentro do ciclo
                    min_dist = new_min_dist; // menor distância
                    min_jit = jit; // vértice j
                    min_point = point; // vértice k
                }
            }
        }
        
        // Insere point (vértice k) antes do vértice j, e após o vértice i encontrado
        if (min_jit != cycle.end()) {
            cycle.insert(min_jit, *min_point);
            points.erase(min_point);
        }
    }
}

/* main
 * argc: int, quantidade de strings contidas na chamada do programa.
 * argv: char**, vetor de strings contendo as informações na chamada do programa.
 * Função inicializadora do programa.
 * Retorna 1 em caso de falha, quando o arquivo de texto de input não é informado na chamada
 * do programa.
 * Retorna 0 em caso de execução com sucesso do programa.
 * 
 * - Análise de complexidade:
 * O algoritmo lê do arquivo os pontos gerados. Isso executa na ordem de O(N), para n linhas no arquivo.
 * Realiza operações de atribuição de tempo, de custo constante.
 * Calcula o fecho convexo com o algoritmo quick_hull, que garante tempo médio e no melhor caso de
 * O(N * lgN), e no pior caso de O(N^2).
 * Escreve o output do fecho, com custo linear de O(N).
 * Caso o o fecho convexo seja um ciclo hamiltoniano, ou seja, o tamanho do fecho seja igual ao tamanho do vetor de pontos
 * analisados, nenhuma outra etapa de cálculo será necessária.
 * Caso contrário:
 *    Converte o vetor de pontos lidos em uma lista encadeada para facilitar a remoção posterior, executando em O(N).
 *    Remove pontos da lista de pontos que já estão no ciclo, e portanto são do fecho convexo, o que têm
 *    custo da ordem de O(N * Ci), com Ci sendo o tamanho do fecho convexo calculado no quick_hull. No melhor caso, quando
 *    Ci é constante, essa operação é linear, no pior, quando Ci é da ordem de N, executa em O(N^2).
 *    Executa a função tsp, que é da ordem de O(N^3).
 * É inserido no fim da lista o começo dela para fechar um ciclo corretamente.
 * Calcula a distância percorrida no ciclo, em tempo linear O(N).
 * Por fim, escreve o output do ciclo, em ordem linear também O(N).
 * 
 * Melhor Caso:
 * Ocorre quando algoritmo quick_hull executa em tempo O(N * lgN) e retorna um ciclo hamiltoniano como fecho convexo.
 * Esse caso é raro, mas é executado em O(N * lgN).
 * 
 * Pior caso e caso comum:
 * Todos os outros casos que não o citado acima são limitados pela complexidade O(N^3), resultado da função tsp.
 *
 * Portanto, salvo o melhor caso raro que executa em O(N * lgN), o algoritmo executa em O(N^3).
 */
int main(int argc, char* argv[]) {
	// Tratamento de erro caso o input seja fora dos padrões definidos
	if (argc < 2) {
		std::cout << "Usage: ./tsp <filename>" << std::endl;
		std::cout << "Example : ./tsp input.txt" << std::endl;
		return 1;
	}
    // Realiza a leitura dos pontos dentro do arquivo especificado
	std::string filename = argv[1];
	std::vector<Point2D> points = read_input(filename);

	// Tempo do início do algoritmo
	auto start = std::chrono::system_clock::now();

    // Calcula o fecho convexo
	std::list<Point2D> hull = quick_hull(points);
    // Escreve o fecho convexo computado dentro de fecho.txt
	write_output("fecho.txt", hull);

    // Se a hull encontrada tiver o mesmo tamanho dos pontos informados, então a hull é um ciclo hamiltoniano.
    if (hull.size() < points.size()) {
        // Converte o vetor de pontos retornado para uma lista encadeada de pontos.
        std::list<Point2D> points_list {std::begin(points), std::end(points)};

        // Aqui começa o tratamento dos dados para realizar o algoritmo do
        // travelling salesman com a heurística de inserção mais barata (cheapest insertion)
        
        // Remove os pontos do ciclo dentro dos pontos lidos do arquivo
        remove_cycle_from_points(hull, points_list);
        
        // Realiza o travelling salesman com a técnica de cheapest insertion para inserir os pontos restantes
        // dentro de points dentro de hull
        tsp(points_list, hull);
    }
    
    // Insere o ponto inicial no final do ciclo, para fechar o ciclo
    hull.push_back(hull.front());

    // Calcula a distância total do percurso definido dentro de hull (em ordem anti-horária)
    float total_dist = calc_dist(hull);

	// Tempo do fim do algoritmo
	auto end = std::chrono::system_clock::now();

    // Escreve o ciclo calculado do travelling salesman dentro de "ciclo.txt"
	write_output("ciclo.txt", hull);

	// Calcula o tempo de execução do algoritmo
	std::chrono::duration<double> elapsed_time = end - start;
    
	// Adiciona precisão de 6 casas no output
	std::cout << std::fixed << std::setprecision(6);
    // Imprime o tempo e a distância calculada
	std::cout << elapsed_time.count() << " " << total_dist << std::endl;

	return 0;
}

