use std::marker::PhantomData;
use std::ops::{Div, Range, Sub};

#[derive(Debug)]
struct QuadTree<Unit, Item> {
    root: QuadTreeNode<Unit, Item>,
    max_items: usize,
}

#[derive(Debug)]
enum QuadTreeNode<Unit, Item> {
    Leaf {
        ranges: [Range<Unit>; 2],
        items: Vec<Item>,
        max_items: usize,
    },
    Parent {
        ranges: [Range<Unit>; 2],
        children: Box<[QuadTreeNode<Unit, Item>; 4]>,
        max_items: usize,
    },
}

#[derive(Debug)]
enum QuadTreeError {
    EmptyRegion,
    NodeAlreadySplit,
    OutOfBounds,
}

trait Two<T> {
    fn two() -> T;
}

impl<T> Two<T> for T
where
    T: From<u32>,
{
    fn two() -> T {
        T::from(2u32)
    }
}

trait MidPoint<Unit> {
    fn mid_point(&self) -> Unit;
}

trait Halveable:
    Sized + PartialOrd + Sub<Output = Self> + Div<Output = Self> + Two<Self>
{
}

impl<T> Halveable for T where
    T: Sized + PartialOrd + Sub<Output = Self> + Div<Output = Self> + Two<Self>
{
}

impl<Unit> MidPoint<Unit> for Range<Unit>
where
    Unit: Halveable + Clone,
{
    fn mid_point(&self) -> Unit {
        (self.end.clone() - self.start.clone()) / Unit::two()
    }
}

impl<Unit, Item> QuadTreeNode<Unit, Item>
where
    Item: PartialOrd<Unit>,
    Unit: Halveable + PartialOrd<Item> + Clone,
    Range<Unit>: MidPoint<Unit>,
{
    fn new(ranges: [Range<Unit>; 2], max_items: usize) -> Self {
        Self::Leaf {
            ranges,
            max_items,
            items: vec![],
        }
    }

    fn insert(&mut self, item: Item) {
        match &mut *self {
            Self::Leaf {
                items, max_items, ..
            } => {
                items.push(item);
                if items.len() > *max_items {
                    if let Err(e) = self.split() {
                        eprintln!("Error splitting node: {e:?}");
                        panic!();
                    }
                }
            }
            Self::Parent { children, .. } => {
                for child in children.iter_mut() {
                    if child
                        .get_ranges()
                        .iter()
                        .all(|range| range.contains::<Item>(&item))
                    {
                        child.insert(item);
                        break;
                    }
                }
            }
        }
    }

    fn get_ranges(&mut self) -> &[Range<Unit>] {
        match self {
            Self::Leaf { ranges, .. } => &ranges[..],
            Self::Parent { ranges, .. } => &ranges[..],
        }
    }

    fn insert_items(&mut self, items: Vec<Item>) {
        items.into_iter().for_each(|item| self.insert(item));
    }

    fn split(&mut self) -> Result<(), QuadTreeError> {
        let self_owned = unsafe { std::ptr::read(self) };
        let mut result = Ok(());
        let new = match self_owned {
            Self::Leaf {
                ranges,
                items,
                max_items,
            } => {
                if !ranges.iter().any(|range| range.is_empty()) {
                    // Dimension is K = 2.
                    // Number of different subranges is 2^K = 4, i.e. 1 << K
                    let children: [QuadTreeNode<Unit, Item>; 1 << 2] =
                        std::array::from_fn(|combination| {
                            let subranges: [Range<Unit>; 2] =
                                std::array::from_fn(|i| {
                                    let range = &ranges[i];
                                    let mid_point = range.mid_point();
                                    if combination & (1 << i) == 0 {
                                        range.start.clone()..mid_point
                                    } else {
                                        mid_point..range.end.clone()
                                    }
                                });
                            // Use subranges to create a child QuadTreeNode
                            QuadTreeNode::new(subranges, max_items)
                        });

                    let mut tmp = Self::Parent {
                        ranges,
                        max_items,
                        children: Box::new(children),
                    };
                    tmp.insert_items(items);
                    tmp
                } else {
                    result = Err(QuadTreeError::EmptyRegion);
                    Self::Leaf {
                        ranges,
                        items,
                        max_items,
                    }
                }
            }
            other => {
                result = Err(QuadTreeError::NodeAlreadySplit);
                other
            }
        };
        unsafe {
            std::ptr::write(self, new);
        };
        result
    }
}

impl<Unit, Item> QuadTree<Unit, Item>
where
    Item: PartialOrd<Unit>,
    Unit: Halveable + PartialOrd<Item> + Clone,
    Range<Unit>: MidPoint<Unit>,
{
    fn new(ranges: [Range<Unit>; 2], max_items: usize) -> Self {
        let root: QuadTreeNode<Unit, Item> = QuadTreeNode::Leaf {
            ranges,
            items: vec![],
            max_items,
        };
        Self { root, max_items }
    }

    fn insert(&mut self, item: Item) -> Result<(), QuadTreeError> {
        if !self
            .root
            .get_ranges()
            .iter()
            .all(|range| range.contains::<Item>(&item))
        {
            Err(QuadTreeError::OutOfBounds)
        } else {
            self.root.insert(item);
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tinkering() {
        let mut tree: QuadTree<f64, f64> =
            QuadTree::new([(0.0..1.0), (0.0..1.0)], 2);
        tree.insert(0.2);
        tree.insert(0.5);
        tree.insert(0.8);
        println!("{tree:#?}");
    }
}
