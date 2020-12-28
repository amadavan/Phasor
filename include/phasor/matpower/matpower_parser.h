#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_PARSER_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_PARSER_H__

#include <filesystem>
#include <fstream>
#include <iostream>

#include <Eigen/Core>

#include "phasor/power/power_network.h"
#include "phasor/matpower/network.h"

namespace phasor::matpower
{
  Network parseFile(const std::string &mfile);
  Eigen::MatrixXd getMatrixFromText(std::ifstream &input);

  class Parser
  {
  public:
    Parser(const std::string &filename) : filename_(filename) {}

    Network parseMatpower()
    {

      if (!std::filesystem::exists(mfile))
        throw std::invalid_argument("File does not exist.");

      int version = 2;
      double baseMVA;
      Eigen::MatrixXd branch, bus, gen, gencost;

      std::ifstream ifile(mfile);

      std::string line;
      while (std::getline(ifile, line))
      {
        if (line[0] == '%')
          continue;

        std::istringstream iss(line);
        std::string header;
        std::getline(iss, header, ' ');

        if (header.compare(0, 3, "mpc") == 0)
        {
          header = header.substr(4);

          std::string value;

          if (header.compare("version") == 0)
          {
            std::getline(iss, value, '=');
            iss.ignore(2);

            iss >> version;
          }
          else if (header.compare("baseMVA") == 0)
          {
            std::getline(iss, value, '=');
            iss.ignore(1);
            std::getline(iss, value, ';');

            baseMVA = std::stod(value);
          }
          else if (header.compare("bus") == 0)
          {
            bus = getMatrixFromText(ifile);
          }
          else if (header.compare("branch") == 0)
          {
            branch = getMatrixFromText(ifile);
          }
          else if (header.compare("gen") == 0)
          {
            gen = getMatrixFromText(ifile);
          }
          else if (header.compare("gencost") == 0)
          {
            gencost = getMatrixFromText(ifile);
          }
        }
      }

      return {
          bus,
          branch,
          gen,
          gencost,
          version,
          baseMVA};
    }

  private:
    const std::string filename_;

    Eigen::MatrixXd getMatrixFromText(std::ifstream &is)
    {
      std::string line;
      size_t n_lines = 0;

      std::vector<std::vector<double>> data;

      double datum;

      while (std::getline(input, line))
      {
        if (line[0] == ']')
          break;

        std::stringstream ss(line);
        std::vector<double> row;

        while (ss >> datum)
          row.push_back(datum);

        data.push_back(row);
      }

      Eigen::MatrixXd mat(data.size(), data[0].size());
      for (size_t i = 0; i < data[0].size(); ++i)
      {
        for (size_t j = 0; j < data.size(); ++j)
          mat.coeffRef(j, i) = data[j][i];
      }
      return mat;
    }
  };

} // namespace phasor::matpower

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_PARSER_H__