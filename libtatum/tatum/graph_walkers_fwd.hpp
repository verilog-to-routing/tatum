#pragma once

namespace tatum {

template<class Visitor>
class SerialWalker;

template<class Visitor>
class ParallelLevelizedCilkWalker;

///The default parallel graph walker
template<class Visitor>
using ParallelWalker = ParallelLevelizedCilkWalker<Visitor>;

} //namespace

