# Liquid 4.0.3 calls Object#tainted?, which Ruby 3.4 removed.
class Object
  def tainted?
    false
  end unless method_defined?(:tainted?)

  def untaint
    self
  end unless method_defined?(:untaint)
end
