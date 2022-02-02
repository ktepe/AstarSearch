#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"
#include "ket_dbg.h"


using namespace std::experimental;
using namespace std;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

static vector<float> *enter_end_points(){
    
  vector<float> *start_end_points;
  
  string  WARNING ("you need to enter a number between [0,100] \n"); //+ std::endl; 
  float start_x, start_y, end_x, end_y;
  
  cout << "you choose to enter new starting and ending points for the route" << std::endl;  
  while(true){
    cout << "enter starting point X in [0,100] coordinate: ";	
    cin >> start_x;
	if (start_x > 100 || start_x < 0) {
    	cout << WARNING;
    } 
    else{
    break;
    }
  }
    while(true){
    cout << "enter start  point Y in [0,100] coordinate: ";	
    cin >> start_y;
	if (start_y > 100 || start_y < 0) {
    	cout << WARNING;
    } 
    else{
    break;
    }
    }
      while(true){
    cout << "enter end  point X in [0,100] coordinate: ";	
    cin >> end_x;
	if (end_x > 100 || end_x < 0) {
    	cout << WARNING;
    }
      else{
    break;
    }
      }
     
  while(true){    
    cout << "enter end  point y in [0,100] coordinate: ";	
    cin >> end_y;
	if (end_y > 100 || end_y < 0) {
    	cout << WARNING;
    } 
    else{
    break;
    }
             }
        start_end_points->push_back(start_x);
        start_end_points->push_back(start_y);
        start_end_points->push_back(end_x);
    	start_end_points->push_back(end_y);
                  
       return start_end_points;
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
   }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else 
            osm_data = std::move(*data);
    }
    //Complete this TODO to satisfy Project Rubric Criterias of User Input
  
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
	// KET
 	float start_x, start_y, end_x, end_y;
  	vector<float> *start_end_points;
  start_end_points = enter_end_points();
  
  end_y = start_end_points->back(); //[3];
  start_end_points->pop_back();
  end_x = start_end_points->back();
  start_end_points->pop_back();
  start_y = start_end_points->back();
  start_end_points->pop_back();
  start_x = start_end_points->back(); 
  start_end_points->pop_back();
  // end of KET
  
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
