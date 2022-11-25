#include <obj.h>

#include <fstream>
#include <iostream>

namespace tnp {

template<typename Out>
void split(const std::string& str, char delim, Out result) 
{
    std::istringstream ss(str);
    std::string token;
    while(std::getline(ss, token, delim)) 
    {
        *result++ = token;
    }
}

std::vector<std::string> split(const std::string& str, char delim = ' ') 
{
    std::vector<std::string> tokens;
    split(str, delim, std::back_inserter(tokens));
    return tokens;
}

bool load_obj(
    const std::string& filename, 
    std::vector<Eigen::Vector3f>& points)
{
    std::vector<Eigen::Vector3f> normals, colors;
    return load_obj(filename, points, normals, colors);
}

bool load_obj(
    const std::string& filename, 
    std::vector<Eigen::Vector3f>& points,
    std::vector<Eigen::Vector3f>& normals)
{
    std::vector<Eigen::Vector3f> colors;
    return load_obj(filename, points, normals, colors);
}

bool load_obj(
    const std::string& filename, 
    std::vector<Eigen::Vector3f>& points,
    std::vector<Eigen::Vector3f>& normals,
    std::vector<Eigen::Vector3f>& colors)
{
    points.clear();
    normals.clear();
    colors.clear();

    std::ifstream fs(filename);
    if(not fs.is_open())
    {
        std::cout << "Error: "
            << "failed to open input obj file '" 
            << filename 
            << "', file not found, nothing loaded" 
            << std::endl;
        return false;
    }

    std::string line;
    for(auto idx_line = 0; std::getline(fs, line); ++idx_line)
    {
        const auto tokens = split(line);
        if(line.empty()) 
        {
            // empty line
            // nothing to do
        }
        else if(line.front() == '#')
        {
            // comment = "# ..."
            // nothing to do
        }
        if(not tokens.empty() and tokens.front() == "v")
        {
            // line = "v x y z"
            // or line = "v x y z r g b"
            if(tokens.size() == 4)
            {
                points.push_back(Eigen::Vector3f{
                    std::stof(tokens[1]),
                    std::stof(tokens[2]),
                    std::stof(tokens[3])
                });
            }
            else if(tokens.size() == 7)
            {
                points.push_back(Eigen::Vector3f{
                    std::stof(tokens[1]),
                    std::stof(tokens[2]),
                    std::stof(tokens[3])
                });
                colors.push_back(Eigen::Vector3f{
                    std::stof(tokens[4]),
                    std::stof(tokens[5]),
                    std::stof(tokens[6])
                });
            }
            else
            {
                std::cout << "Warning: "
                    << "failed to read line " 
                    << idx_line 
                    << " of input obj file '" 
                    << filename 
                    << "', 3 or 6 values expected but "
                    << tokens.size()-1 // -1 for the front token (v) 
                    << " read instead, line skipped" 
                    << std::endl;
            }
        }
        else if(not tokens.empty() and tokens.front() == "vn")
        {
            // line = "vn nx ny nz"
            if(tokens.size() == 4)
            {
                normals.push_back(Eigen::Vector3f{
                    std::stof(tokens[1]),
                    std::stof(tokens[2]),
                    std::stof(tokens[3])
                });
            }
            else
            {
                std::cout << "Warning: "
                    << "failed to read line " 
                    << idx_line 
                    << " of input obj file '" 
                    << filename 
                    << "', 3 values expected but "
                    << tokens.size()-1 // -1 for the front token (vn) 
                    << " read instead, line skipped" 
                    << std::endl;
            }
        }
        else
        {
            std::cout << "Warning: " 
                << "failed to read line " 
                << idx_line 
                << " of input obj file '" 
                << filename 
                << "', 'v' or 'vn' expected but '" 
                << tokens.front()
                << "' read instead, line skipped"
                << std::endl;
        }
    } // end of loop

    if(points.size() == 0) 
    {
        std::cout << "Error:"
            << "no points read from input obj file '" 
            << filename 
            << "'" 
            << std::endl;
        return false;
    }
    if(not normals.empty() and normals.size() != points.size())
    {
        std::cout << "Warning: " 
            << "failed to read normals from input obj file '" 
            << filename 
            << "', "
            << points.size() 
            << " expected but " 
            << normals.size() 
            << " read, normals cleared"
            << std::endl;
        normals.clear();
    }
    if(not colors.empty() and colors.size() != points.size())
    {
        std::cout << "Warning: " 
            << "failed to read colors from input obj file '" 
            << filename 
            << "', " 
            << points.size() 
            << " expected but " 
            << colors.size() 
            << " read, normals cleared"
            << std::endl;
        colors.clear();
    }
    std::cout << "Loaded " 
        << points.size() 
        << " points from obj file '" << filename << "'";
    if(not normals.empty() and not colors.empty()) 
        std::cout << " (with normals and colors)";
    else if(not normals.empty()) 
        std::cout << " (with normals)";
    else if(not colors.empty()) 
        std::cout << " (with colors)";
    std::cout << std::endl;
    return true;
}

bool save_obj(
    const std::string& filename, 
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3i>& faces)
{
    return save_obj(filename, points, {}, {}, faces);
}

bool save_obj(
    const std::string& filename, 
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3f>& normals,
    const std::vector<Eigen::Vector3f>& colors)
{
    return save_obj(filename, points, normals, colors, {});
    // std::ofstream fs(filename);
    // if(not fs.is_open())
    // {
    //     std::cout << "Error: "
    //         << "failed to open output obj file '" 
    //         << filename 
    //         << "', file not found, nothing saved" 
    //         << std::endl;
    //     return false;
    // }

    // if(points.size() == 0)
    // {
    //     std::cout << "Warning: "
    //         << "saved 0 points to output obj file '" 
    //         << filename
    //         << "'" 
    //         << std::endl;
    //     return true;
    // }

    // const auto save_normals = (not normals.empty()) and normals.size() == points.size();
    // const auto save_colors  = (not colors.empty())  and colors.size()  == points.size(); 

    // if((not normals.empty()) and normals.size() != points.size())
    // {
    //     std::cout << "Warning: "
    //         << "normals size (" 
    //         << normals.size()
    //         << ") is different from points size (" 
    //         << points.size() 
    //         << "), normals not saved to output obj file '" 
    //         << filename 
    //         << "'" 
    //         << std::endl;
    // }
    // if((not colors.empty()) and colors.size() != points.size())
    // {
    //     std::cout << "Warning: "
    //         << "colors size (" 
    //         << colors.size()
    //         << ") is different from points size (" 
    //         << points.size() 
    //         << "), colors not saved to output obj file '" 
    //         << filename
    //         << "'" 
    //         << std::endl;
    // }

    // for(auto i = 0u; i < points.size(); ++i)
    // {
    //     fs << "v " 
    //         << points[i].x() << ' '
    //         << points[i].y() << ' '
    //         << points[i].z();
    //     if(save_colors)
    //     {
    //         fs << ' '
    //             << colors[i].x() << ' '
    //             << colors[i].y() << ' '
    //             << colors[i].z();
    //     }
    //     fs << '\n';
    // }
    // if(save_normals)
    // {
    //     for(auto i = 0u; i < normals.size(); ++i)
    //     {
    //         fs << "vn " 
    //             << normals[i].x() << ' '
    //             << normals[i].y() << ' '
    //             << normals[i].z() << '\n';
    //     }   
    // }
    // std::cout << "Saved " 
    //     << points.size() 
    //     << " points to obj file '" << filename << "'";
    // if(save_normals and save_colors) 
    //     std::cout << " (with normals and colors)";
    // else if(save_normals) 
    //     std::cout << " (with normals)";
    // else if(save_colors) 
    //     std::cout << " (with colors)";
    // std::cout << std::endl;
    // return true;
}


bool save_obj(
    const std::string& filename, 
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3f>& normals,
    const std::vector<Eigen::Vector3i>& faces)
{
    return save_obj(filename, points, normals, {}, faces);
    // std::ofstream fs(filename);
    // if(not fs.is_open())
    // {
    //     std::cout << "Error: "
    //         << "failed to open output obj file '" 
    //         << filename 
    //         << "', file not found, nothing saved" 
    //         << std::endl;
    //     return false;
    // }

    // if(points.size() == 0)
    // {
    //     std::cout << "Warning: "
    //         << "saved 0 points to output obj file '" 
    //         << filename
    //         << "'" 
    //         << std::endl;
    //     return true;
    // }

    // const auto save_normals = (not normals.empty()) and normals.size() == points.size();

    // if((not normals.empty()) and normals.size() != points.size())
    // {
    //     std::cout << "Warning: "
    //         << "normals size (" 
    //         << normals.size()
    //         << ") is different from points size (" 
    //         << points.size() 
    //         << "), normals not saved to output obj file '" 
    //         << filename 
    //         << "'" 
    //         << std::endl;
    // }

    // for(auto i = 0u; i < points.size(); ++i)
    // {
    //     fs << "v " 
    //         << points[i].x() << ' '
    //         << points[i].y() << ' '
    //         << points[i].z();
    //     fs << '\n';
    // }
    // if(save_normals)
    // {
    //     for(auto i = 0u; i < normals.size(); ++i)
    //     {
    //         fs << "vn " 
    //             << normals[i].x() << ' '
    //             << normals[i].y() << ' '
    //             << normals[i].z() << '\n';
    //     }   
    // }
    // if(not faces.empty())
    // {
    //     for(auto i = 0u; i < faces.size(); ++i)
    //     {
    //         // +1 because obj indices start at 1!
    //         fs << "f "
    //             << faces[i][0]+1 << ' '
    //             << faces[i][1]+1 << ' '
    //             << faces[i][2]+1 << '\n';
    //     }
    // }

    // std::cout << "Saved " 
    //     << points.size() 
    //     << " points to obj file '" << filename << "'";
    // if(save_normals) 
    //     std::cout << " (with normals)";
    // if(not faces.empty())
    //     std::cout << " (with " << faces.size() << " faces)";
    // std::cout << std::endl;
    // return true;
}


bool save_obj(
    const std::string& filename, 
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3f>& normals,
    const std::vector<Eigen::Vector3f>& colors,
    const std::vector<Eigen::Vector3i>& faces)
{
    std::ofstream fs(filename);
    if(not fs.is_open())
    {
        std::cout << "Error: "
            << "failed to open output obj file '" 
            << filename 
            << "', file not found, nothing saved" 
            << std::endl;
        return false;
    }

    if(points.size() == 0)
    {
        std::cout << "Warning: "
            << "saved 0 points to output obj file '" 
            << filename
            << "'" 
            << std::endl;
        return true;
    }

    const auto save_normals = (not normals.empty()) and normals.size() == points.size();
    const auto save_colors  = (not colors.empty())  and colors.size()  == points.size(); 

    if((not normals.empty()) and normals.size() != points.size())
    {
        std::cout << "Warning: "
            << "normals size (" 
            << normals.size()
            << ") is different from points size (" 
            << points.size() 
            << "), normals not saved to output obj file '" 
            << filename 
            << "'" 
            << std::endl;
    }
    if((not colors.empty()) and colors.size() != points.size())
    {
        std::cout << "Warning: "
            << "colors size (" 
            << colors.size()
            << ") is different from points size (" 
            << points.size() 
            << "), colors not saved to output obj file '" 
            << filename
            << "'" 
            << std::endl;
    }

    for(auto i = 0u; i < points.size(); ++i)
    {
        fs << "v " 
            << points[i].x() << ' '
            << points[i].y() << ' '
            << points[i].z();
        if(save_colors)
        {
            fs << ' '
                << colors[i].x() << ' '
                << colors[i].y() << ' '
                << colors[i].z();
        }
        fs << '\n';
    }
    if(save_normals)
    {
        for(auto i = 0u; i < normals.size(); ++i)
        {
            fs << "vn " 
                << normals[i].x() << ' '
                << normals[i].y() << ' '
                << normals[i].z() << '\n';
        }   
    }
    if(not faces.empty())
    {
        for(auto i = 0u; i < faces.size(); ++i)
        {
            // +1 because obj indices start at 1!
            fs << "f "
                << faces[i][0]+1 << ' '
                << faces[i][1]+1 << ' '
                << faces[i][2]+1 << '\n';
        }
    }

    std::cout << "Saved " 
        << points.size() 
        << " points to obj file '" << filename << "'";
    if(save_normals and save_colors) 
        std::cout << " (with normals and colors)";
    else if(save_normals) 
        std::cout << " (with normals)";
    else if(save_colors) 
        std::cout << " (with colors)";
    if(not faces.empty())
        std::cout << " (with " << faces.size() << " faces)";
    std::cout << std::endl;
    return true;
}

} // namespace tnp