// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

namespace fieldfriend_driver
{

/// Common base so heterogeneous module handlers can be stored/owned together.
class ModuleHandler
{
public:
  virtual ~ModuleHandler() = default;
};

}  // namespace fieldfriend_driver
