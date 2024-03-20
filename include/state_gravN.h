#ifndef STATE_GRAVN_H
#define STATE_GRAVN_H

#include "state.h"

class GravityPoints : public State {
private:
    static int id_to_index(int id,int dim);
    static int index_to_id(int id);
    static double grav_profile(double dist);
    int numPoints() const;
public:
    GravityPoints();
    GravityPoints(std::vector<double> new_data);
    GravityPoints(std::vector<double> new_data_0, std::vector<double> new_data_1);
    GravityPoints(const GravityPoints &other);
    GravityPoints& operator=(const GravityPoints &other);

    virtual void solve_deltas(GravityPoints &output, double time) const;
    virtual void solve_interactions(GravityPoints &output) const;
    virtual void render(SDL_Renderer* sdlr) const override;

};

#endif // STATE_GRAVN_H