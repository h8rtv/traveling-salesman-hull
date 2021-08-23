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
 * será executado Ci vezes, onde Ci é o número de pontos dentro do ciclo.
 * O segundo laço de repetição, que está dentro do primeiro, será executado N vezes, porém como o mesmo
 * está dentro do primeiro laço, ele será executado no máximo N*C vezes. Em média, assumindo que o ponto
 * a ser buscado está no meio da lista, o custo médio será Ci * (N/2), pois o loop interno para a iteração
 * quando encontra o valor buscado. As operações executadas dentro desse loop são de tempo constante.
 * Logo, a complexidade de remove_cycle_from_points é limitada por O(N*Ci).
 * 
 * Melhor caso:
 * Quando Ci e constante, o custo de O(N*Ci) será linear da ordem de O(N).
 * 
 * Pior caso:
 * Para N grande e uma distribuição uniforme, Ci é em geral bem pequeno, mas no pior caso, quando o fecho convexo é
 * muito próximo de um ciclo hamiltoniano, pode ser da ordem de N. Neste caso, o custo do algoritmo será da ordem de O(N^2).
 *
 * Caso médio:
 * Assumindo uma distribuição normal de pontos, percebe-se um N que domina o tamanho do fecho convexo encontrado.
 * Com base nisso, pode-se dizer que o crescimento de N supera o crescimento de Ci. Portanto, se assumirmos que
 * Ci é um fator de de N na forma de Ci = N * f, 0 < f < 1, a fórmula resultante será:
 * (N*N*f), que também é da ordem de O(N^2).
 * Por causa disso, a execução média também é O(N^2).
 *
 * - Corretude:
 * 
 * Prova por loop invariante:
 * 
 * Invariante de loop: O subvetor de points Pa não conterá os pontos que fazem parte do ciclo C.
 * 
 * Inicialização:
 * |Pa| é zero, e portanto, nenhum ponto foi removido dentro de points.
 * 
 * Pa = []
 * 
 * Manutenção:
 * Ao fim da iteração do laço, foi comparado o ponto do ciclo da iteração atual com os pontos dentro
 * de points e removido o ponto da iteração dentro da lista de points. Portanto, se o ponto i analisado
 * estiver em C, este será removido, mantendo a invariante.
 * 
 * Pa = P[1:i]
 * 
 * Término:
 * Após o final do loop, o subvetor Pa conterá apenas os pontos que não estiverem dentro do ciclo
 * analisado.
 * 
 * Pa = P[1:N]
 * 
 * Portanto, assumindo que A seja a lista de pontos e B seja a lista de pontos dentro do ciclo,
 * a lista de points conterá A - B.
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
 * points: std::list<Point2D>&, lista de pontos que não estão no ciclo ainda. 
 * cycle: std::list<Point2D>&, fecho convexo previamente calculado.
 * Função que adiciona os pontos da lista points na lista cycle de forma a construir um
 * ciclo hamiltoniano, utilizado para retornar uma solução aproximada do
 * problema do caixeiro viajante (tsp).
 * 
 * - Análise de complexidade:
 *
 * O algoritmo inicia iterando por todos os pontos da lista points, com um custo de O(N - Ci), sendo Ci o 
 * tamanho do fecho convexo calculado previamente.
 * Dentro do loop, executa operações constantes e outro loop, que percorre o ciclo do feixo
 * convexo, executanto O(N * C) vezes, com C sendo o tamanho do ciclo atual. O tamanho do ciclo 
 * aumenta com cada iteração do loop, pois um novo ponto é adicionado a cada iteração. Portanto,
 * Ci sendo tamanho do fecho convexo inicial, o loop interno executa Ci + (Ci + 1) + (Ci + 2) +
 * + (Ci + 3) + ... + (N - 2) + (N - 1) + N, o que implica em:
 * N(N + 1) / 2 - Ci(Ci + 1) / 2 = (N^2)/2 + N/2 - (Ci^2)/2 - Ci/2
 * Dentro do loop interno, são executadas operações constantes. 
 * Ao fim do loop interno, é feita uma inserção em lista encadeada, com custo constante também.
 *
 * Melhor caso:
 * Ocorre quando N = Ci + 1, com Ci sendo o tamanho do fecho convexo.
 * Neste caso, o tamanho da lista de pontos é 1 e portanto o loop externo executa somente uma vez.
 * O loop interno executará na ordem de N e portanto o algoritmo se dará em O(N).
 * É importante notar que o verdadeiro melhor caso, quando N = Ci, não executa a função tsp, pois este
 * pode ser facilmente identificado previamente com um analise do tamanho do vetor de pontos com a lista
 * do fecho convexo. Caso sejam iguais o fecho convexo é um ciclo hamiltoniano.
 * 
 * Pior caso:
 * Ocorre quando Ci <= 3. Neste caso, o loop externo executará na ordem O(N) e o loop interno também, resultando
 * num algoritmo da ordem de O(N^2).
 *
 * Caso médio:
 * Assumindo uma distribuição normal de pontos, percebe-se um N que domina o tamanho do fecho convexo encontrado.
 * Com base nisso, pode-se dizer que o crescimento de N supera o crescimento de Ci. Portanto, se assumirmos que
 * Ci é um fator de de N na forma de Ci = N * f, 0 < f < 1, a fórmula resultante será:
 * (N^2)/2 + N/2 - ((N*f)^2)/2 - (N*f)/2, que também é da ordem de O(N^2).
 * Por causa disso, a execução média também é O(N^2).
 *
 * - Corretude:
 * Definição:
 *  Um ciclo hamiltoniano é um ciclo que passa por todos os vértices de um grafo somente uma vez.
 *  Todo grafo completo com 2 ou mais elementos possui um ciclo hamiltoniano. Portanto, como o conjunto de pontos
 *  analisados pode ser visto como um grafo completo, sempre pode-se encontrar um ciclo hamiltoniano.
 * 
 * Invariante de loop:
 *  Dado um grafo completo G(E, V),
 *  C sempre será um ciclo hamiltoniano do conjunto de vértices analisados Va que está contido em V.
 * 
 * Inicialização:
 *  C contém os pontos do fecho convexo, que por definição é um ciclo.
 *  Os pontos analisados em Va são aqueles retornados pelo fecho convexo.
 *  
 *  C = a, b, c, ... n, onde C é o caminho que representa o fecho convexo.
 *  Va contém todos os pontos de C
 * 
 * Manutenção:
 *  Dado uma aresta A(i, j) dentro do ciclo que pertence a E e um vértice k que pertence a V e está fora do ciclo C,
 *  para cada k, escolhe-se sempre o que minimiza a operação: dik + djk - dij, sendo
 *  dik a distância do vértice i até o vértice k, ou o peso da aresta IK
 *  djk a distância do vértice j até o vértice k, ou o peso da aresta JK
 *  dij a distância do vértice i até o vértice i, ou o peso da aresta E
 *  Uma vez que é encontrado uma tripla (i, j, k) que minimiza a operação:
 *   1. A aresta A é removida do ciclo C
 *   2. A aresta IK é inserida no ciclo C
 *   3. A aresta KJ é inserida no ciclo C
 *  
 * Ao substituir A por IK e KJ, mantém-se o ciclo hamiltoniano em Va, pois os vértices previamente inseridos I e J se mantém no ciclo
 * e K é adicionado somente uma vez.
 * 
 *  C = a, b, c, ... i, k, j, ... n, onde k é o ponto inserido nessa iteração.
 *  Va contém todos os pontos de C
 * 
 * Término:
 * Após o fim do loop, todos os vértices em V foram analisados e inseridos em C.
 * 
 *  C = a, b, c, ... n, onde C contém todos os pontos de V.
 *  Va contém todos os pontos de V
 * 
 * Portanto, prova-se que o algoritmo irá retornar um ciclo hamiltoniano como resultado que usa uma aproximação de minimizar a inserção pelo
 * custo da aresta.
 */
void tsp(const std::list<Point2D>& points, std::list<Point2D>& cycle) {
    // Para cada ponto não adicionado dentro do ciclo...
    for (const Point2D& point: points) {

        // Variáveis para guardar o ponto com menor distância dentro do ciclo
        float min_dist = INF;
        std::list<Point2D>::iterator min_jit;
        
        // Loop para calcular o ponto com distância entre point e um ponto de cycle
        for (auto iit = cycle.begin(); iit != cycle.end(); ++iit) {
            auto jit = std::next(iit); // Pega o sucessor do ponto i dentro do ciclo
            
            if (jit == cycle.end()) { // Se o ponto j não existir...
                jit = cycle.begin(); // O ponto j será o começo do ciclo, para fechar o ciclo
            }
            
            // Aqui o ponto j será o sucessor de i
            
            // Retorna os pontos i e j apontados por seus iteradores
            Point2D& i = *iit,
                     j = *jit;
            
            // Calcula distância entre i-k, j-k e i-j
            // k é o ponto do loop externo, point
            float dik = dist(i, point);
            float djk = dist(j, point);
            float dij = dist(i, j);
            float new_min_dist = dik + djk - dij;

            // Se a distância for menor do que alguma já calculada...
            if (new_min_dist < min_dist) {
                // Substitui com informações para realizar a inserção dentro do ciclo
                min_dist = new_min_dist;
                min_jit = jit;
            }
        }
        
        // Insere point antes do ponto J, e após o ponto I encontrado
        if (min_jit != cycle.end()) {
            cycle.insert(min_jit, point);
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
 *    Executa a função tsp, que no pior caso e no caso médio é O(N^2). No melhor caso ele será O(N), quando o tamanho do
 *    fecho + 1 é igual ao número de pontos analisados N, e no pior caso é de O(N^2) assim como no caso médio.
 *    Portanto, sempre dentro dessa condição terá um custo da ordem de O(N^2).
 * É inserido no fim da lista o começo dela para fechar um ciclo corretamente.
 * Calcula a distância percorrida no ciclo, em tempo linear O(N).
 * Por fim, escreve o output do ciclo, em ordem linear também O(N).
 * 
 * Melhor Caso:
 * Ocorre quando algoritmo quick_hull executa em tempo O(N * lgN) e retorna um ciclo hamiltoniano como fecho convexo.
 * Esse caso é raro, mas é executado em O(N * lgN).
 * 
 * Outros casos possíveis:
 * 1. Ocorre quando o algoritmo de quick_hull executa em O(N^2), não importando os outros casos. Esse caso é limitado por
 * O(N^2).
 * 2. Ocorre quando a remoção dos pontos que estão dentro do fecho convexo executa em O(N^2), limitando a execução a O(N^2).
 * 3. Ocorre quando o cáculo do tsp executa em O(N^2), limitando também a execução em O(N^2).
 * É importante notar que não é possível ocorrer o melhor caso da remoção dos pontos ao mesmo tempo que o melhor caso da função
 * tsp, pois são opostos. (Se o tamanho do fecho convexo for mais próximo do tamanho da lista de pontos analisados,
 * maior o custo da remoção de pontos, e se o tamanho do fecho convexo for o menor possível, maior do tsp)
 *
 * Portanto, salvo o melhor caso raro que executa em O(N * lgN), o algoritmo executa em O(N^2).
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

    // Calcula hull
	std::list<Point2D> hull = quick_hull(points);
    // Escreve o convex hull computado dentro de fecho.txt
	write_output("fecho.txt", hull);

    // Se a hull encontrada tiver o mesmo tamanho dos pontos informados, então a hull é um ciclo hamiltoniano.
    if (hull.size() < points.size()) {
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

