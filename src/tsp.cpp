#include "quick_hull.cpp"
#include <algorithm>

float dist(const Point2D& p1, const Point2D& p2) {
    return sqrt(
        (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
    );
}

float tsp(const std::vector<Point2D>& points, std::list<Point2D>& cycle) {
    for (const Point2D& point: points) {
        float min_dist = INF;
        std::list<Point2D>::iterator min_jit;
        for (auto iit = cycle.begin(); iit != cycle.end(); ++iit) {
            auto jit = std::next(iit);
            if (jit == cycle.end()) {
                jit = cycle.begin();
            }
            Point2D& i = *iit,
                     j = *jit;
            float dik = dist(i, point);
            float djk = dist(j, point);
            float dij = dist(i, j);
            float new_min_dist = dik + djk - dij;

            if (new_min_dist < min_dist) {
                min_dist = new_min_dist;
                min_jit = jit;
            }
        }
        // Insere point antes do ponto J, e após o ponto I
        if (min_jit != cycle.end()) {
            cycle.insert(min_jit, point);
        }
    }

    float total_dist = 0;
    for (auto iit = cycle.begin(); iit != cycle.end(); ++iit) {
        auto jit = std::next(iit);
        if (jit == cycle.end()) {
            jit = cycle.begin();
        }
        Point2D& i = *iit,
                 j = *jit;
        total_dist += dist(i, j);
    }
    
    cycle.push_back(cycle.front());

    return total_dist;
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
 * 
 */
int main(int argc, char* argv[]) {
	// Tratamento de erro caso o input seja fora dos padrões definidos
	if (argc < 2) {
		std::cout << "Usage: ./tsp <filename>" << std::endl;
		std::cout << "Example : ./tsp input.txt" << std::endl;
		return 1;
	}
	std::string filename = argv[1];
	std::vector<Point2D> points = read_input(filename);

	// Tempo do início do algoritmo
	auto start = std::chrono::system_clock::now();

    // Calcula hull
	std::list<Point2D> hull = quick_hull(points);

    // Remove pontos do ciclo da hull
    for (Point2D& cyclePoint : hull) {
        points.erase(std::remove_if(points.begin(), points.end(), [cyclePoint](Point2D point) {
            return point == cyclePoint;
        }), points.end());
    }

	write_output("fecho.txt", hull);

    float total_dist = tsp(points, hull);

	// Tempo do fim do algoritmo
	auto end = std::chrono::system_clock::now();

	write_output("ciclo.txt", hull);

	// Calcula o tempo de execução do algoritmo
	std::chrono::duration<double> elapsed_time = end - start;
	// Adiciona precisão de 6 casas no output
	std::cout << std::fixed << std::setprecision(6);
	std::cout << elapsed_time.count() << " " << total_dist << std::endl;

	return 0;
}

