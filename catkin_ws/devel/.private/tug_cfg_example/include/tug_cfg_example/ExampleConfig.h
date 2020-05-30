#ifndef TUG_CFG_EXAMPLE_EXAMPLECONFIG_H
#define TUG_CFG_EXAMPLE_EXAMPLECONFIG_H

#include <limits>
#include <tug_cfg/scalar.h>
#include <tug_cfg/struct.h>

namespace tug_cfg_example
{
class ExampleConfig
{
public:
  /**
   * Please make sure these are very random
   */
  std::vector<int> pseudo_random_numbers;

  /**
   * Only switch this on if you're absolutely sure what you're doing
   * 
   * Default: false
   */
  bool insane_mode;

  /**
   * Covariance between enigineers (1..num_engineers) and
   * pseudo_random_numbers
   */
  std::vector<std::vector<int> > covariance;

  /**
   * How many software engineers are needed to change a light bulb
   * 
   * Default: 3
   */
  int num_engineers;

  /**
   * Name of each engineer (1..num_engineer) (optional)
   */
  std::map<int, std::string> engineer_names;

  /**
   * Rate of software decay
   * 
   * Unit: 1/s
   * Default: 0.3
   * Minimum: 1e-09
   * Maximum: 1.0
   */
  double decay_rate;

  /**
   * Choosing a proper name is often hard
   * 
   * Default: "foo"
   * Choices: "foo", "bar", "baz"
   */
  std::string var_name;

  /**
   * Put the pedal to the metal
   * 
   * Unit: m/s
   * Default: 0.1
   */
  double max_speed;

  /**
   * Whatever you'd like to add
   */
  std::map<std::string, std::string> extra_params;


  //============================================================================
  // Magic starts here
  //============================================================================

  typedef tug_cfg::Struct<ExampleConfig> MetaObject;

  ExampleConfig()
    : insane_mode(false),
      num_engineers(3),
      decay_rate(0.3),
      var_name("foo"),
      max_speed(0.1),
      meta_object_(*this)
  {
  }

  operator MetaObject&()
  {
    return meta_object_;
  }

  operator const MetaObject&() const
  {
    return meta_object_;
  }

  static const MetaObject::Type& getDefinition()
  {
    using namespace tug_cfg;

    static MetaObject::FieldImpl<Sequence<Scalar<int>>> pseudo_random_numbers_(&ExampleConfig::pseudo_random_numbers, {
      "pseudo_random_numbers", "", "Please make sure these are very random", false, 0, false
    });
    static MetaObject::FieldImpl<Scalar<bool>> insane_mode_(&ExampleConfig::insane_mode, {
      "insane_mode", "", "Only switch this on if you\'re absolutely sure what you\'re doing", true, 0, false,
      false, false, true, {}, {}
    });
    static MetaObject::FieldImpl<Sequence<Sequence<Scalar<int>>>> covariance_(&ExampleConfig::covariance, {
      "covariance", "", "Covariance between enigineers (1..num_engineers) and pseudo_random_numbers", false, 0, false
    });
    static MetaObject::FieldImpl<Scalar<int>> num_engineers_(&ExampleConfig::num_engineers, {
      "num_engineers", "", "How many software engineers are needed to change a light bulb", true, 0, false,
      3, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), {}, {}
    });
    static MetaObject::FieldImpl<Map<int, Scalar<std::string>>> engineer_names_(&ExampleConfig::engineer_names, {
      "engineer_names", "", "Name of each engineer (1..num_engineer) (optional)", false, 0, false
    });
    static MetaObject::FieldImpl<Scalar<double>> decay_rate_(&ExampleConfig::decay_rate, {
      "decay_rate", "1/s", "Rate of software decay", true, 0, false,
      0.3, 1e-09, 1.0, {}, {}
    });
    static MetaObject::FieldImpl<Scalar<std::string>> var_name_(&ExampleConfig::var_name, {
      "var_name", "", "Choosing a proper name is often hard", true, 0, false,
      "foo", "", "", {"foo", "bar", "baz"}, {}
    });
    static MetaObject::FieldImpl<Scalar<double>> max_speed_(&ExampleConfig::max_speed, {
      "max_speed", "m/s", "Put the pedal to the metal", false, 0, false,
      0.1, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}, {}
    });
    static MetaObject::FieldImpl<Map<std::string, Scalar<std::string>>> extra_params_(&ExampleConfig::extra_params, {
      "extra_params", "", "Whatever you\'d like to add", false, 0, false
    });

    static MetaObject::Type type("tug_cfg_example/ExampleConfig", {
      &pseudo_random_numbers_, &insane_mode_, &covariance_, &num_engineers_,
      &engineer_names_, &decay_rate_, &var_name_, &max_speed_, &extra_params_
    });

    return type;
  }

protected:
  MetaObject meta_object_;
};
}  // namespace tug_cfg_example

#endif  // TUG_CFG_EXAMPLE_EXAMPLECONFIG_H
