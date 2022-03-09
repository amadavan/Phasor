#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_PARSER_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_PARSER_H__

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <utility>

#include "phasor/matpower/network.h"

namespace phasor::matpower {
class Parser {
 public:
  explicit Parser(std::string filename) : filename_(std::move(filename)) {}

  Network parseMatpower() {

    int version = 2;
    double baseMVA;
    Eigen::MatrixXd branch, bus, gen, gencost;

    std::ifstream ifile(filename_);

    std::string line;
    while (std::getline(ifile, line)) {
      if (line[0] == '%')
        continue;

      std::istringstream iss(line);
      std::string header;
      std::getline(iss, header, ' ');

      if (header.compare(0, 3, "mpc") == 0) {
        header = header.substr(4);

        std::string value;

        if (header == "version") {
          std::getline(iss, value, '=');
          iss.ignore(2);

          iss >> version;
        } else if (header == "baseMVA") {
          std::getline(iss, value, '=');
          iss.ignore(1);
          std::getline(iss, value, ';');

          baseMVA = std::stod(value);
        } else if (header == "bus") {
          bus = getMatrixFromText(ifile);
        } else if (header == "branch") {
          branch = getMatrixFromText(ifile);
        } else if (header == "gen") {
          gen = getMatrixFromText(ifile);
        } else if (header == "gencost") {
          gencost = getMatrixFromText(ifile);
        }
      }
    }

    return {
        branch,
        bus,
        gen,
        gencost,
        version,
        baseMVA};
  }

 private:
  const std::string filename_;

  static Eigen::MatrixXd getMatrixFromText(std::ifstream &is) {
    std::string line;
    size_t n_lines = 0;

    std::vector<std::vector<double>> data;

    double datum;

    while (std::getline(is, line)) {
      if (line[0] == ']')
        break;

      std::stringstream ss(line);
      std::vector<double> row;

      while (ss >> datum)
        row.push_back(datum);

      data.push_back(row);
    }

    Eigen::MatrixXd mat(data.size(), data[0].size());
    for (size_t i = 0; i < data[0].size(); ++i) {
      for (size_t j = 0; j < data.size(); ++j)
        mat.coeffRef(j, i) = data[j][i];
    }
    return mat;
  }
};

} // namespace phasor::matpower

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_PARSER_H__
