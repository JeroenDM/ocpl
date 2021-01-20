#include <iostream>
#include <vector>
#include <cassert>

#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/random_sampler.h>

using namespace std;
using namespace ocpl;

typedef std::vector<double> Point;

struct Box
{
    const size_t dim;
    const Point centre;
    const vector<double> sides;

    double volume() const
    {
        double volume{ 1.0 };
        for (double side : sides)
            volume *= side;
        return volume;
    }
};

/** \brief A rectangle with the left lower corner in the origin. **/
template <size_t SIZE>
struct Rect
{
    std::array<double, SIZE> sides;

    double volume() const
    {
        double v{ 0.0 };
        for (double s : sides)
            v += s;
        return v;
    }

    bool containsPoint(const Point& p)
    {
        assert(p.size() == sides.size());

        for (size_t d = 0; d < sides.size(); ++d)
        {
            if (p[d] < 0.0 || p[d] >= sides[p])  // use halfopen interval
                return false;
        }
        return true;
    }
};

/** calculate discrepancy
 *
 * Building blocks
 * - Check if a sample fall's inside a subset ( let's use a simple square).
 *
 * */

/** \brief Is a point inside an N-dimensional hypercube. **/
bool isInBox(Point p, const Box& b)
{
    assert(p.size() == b.dim);

    for (size_t d = 0; d < b.dim; ++d)
    {
        double lower = b.centre[d] - b.sides[d] / 2.0;
        double upper = b.centre[d] + b.sides[d] / 2.0;
        if (p[d] < lower || p[d] >= upper)  // use halfopen interval
            return false;
    }
    return true;
}

double discrepancy(const vector<Point>& samples, const Box& box)
{
    size_t num_inside{ 0 };
    for (auto p : samples)
    {
        if (isInBox(p, box))
            num_inside++;
    }

    return ((double)num_inside) / ((double)samples.size());
}

int main()
{
    // GridSampler sampler;
    // sampler.addDimension(0, 1, 10);
    // sampler.addDimension(0, 1, 10);

    RandomSampler sampler;
    sampler.addDimension(0, 1);
    sampler.addDimension(0, 1);

    auto samples = sampler.getSamples(1000);

    double max_value = { 0.0 };
    for (double size{ 0.1 }; size < 1.0; size += 0.1)
    {
        // create a box in the lower left corner of varying sizes.
        const Box box{ 2, { size / 2.0, size / 2.0 }, { size, size } };

        double volume_fraction = discrepancy(samples, box);
        double vol = box.volume();  // the complete sample region has a volume of 1.0

        cout << "Volume fractions for a single box: " << volume_fraction;
        cout << " vs " << vol << "\n";

        double d = std::abs(volume_fraction - vol);

        if (d > max_value)
            max_value = d;
    }

    cout << "  Discrepancy: " << max_value << "\n";

    return 0;
}
