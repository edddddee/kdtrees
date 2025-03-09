//#![feature(test)]
#![feature(generic_const_exprs)]

use std::ops::{Div, Range, Sub};

#[derive(Debug)]
pub struct KdTree<Unit, Item, const K: usize>
where
    [(); 1 << K]: Sized,
{
    pub root: KdTreeNode<Unit, Item, K>,
    pub max_items: usize,
}

#[derive(Debug)]
enum KdTreeNode<Unit, Item, const K: usize>
where
    [(); 1 << K]: Sized,
{
    Leaf {
        ranges: [Range<Unit>; K],
        items: Vec<Item>,
        max_items: usize,
    },
    Parent {
        ranges: [Range<Unit>; K],
        children: Box<[KdTreeNode<Unit, Item, K>; 1 << K]>,
        max_items: usize,
    },
}

#[derive(Debug)]
pub enum KdTreeError {
    EmptyRegion,
    NodeAlreadySplit,
    OutOfBounds,
}

pub trait Two<T> {
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

pub trait Halveable:
    Sized + PartialOrd + Sub<Output = Self> + Div<Output = Self> + Two<Self>
{
}

impl<T> Halveable for T where
    T: Sized + PartialOrd + Sub<Output = Self> + Div<Output = Self> + Two<Self>
{
}

pub trait MidPoint<Unit> {
    fn mid_point(&self) -> Unit;
}

impl<Unit> MidPoint<Unit> for Range<Unit>
where
    Unit: Halveable + Clone,
{
    fn mid_point(&self) -> Unit {
        (self.end.clone() - self.start.clone()) / Unit::two()
    }
}

pub trait Volume<T> {
    fn covers(&self, item: &T) -> bool;
}

pub trait AsCoordinate<T, const K: usize> {
    fn as_coordinate(&self) -> [&T; K];
}

impl<T, const K: usize> AsCoordinate<T, K> for [T; K] {
    fn as_coordinate(&self) -> [&T; K] {
        std::array::from_fn(|i| &self[i])
    }
}

impl<Unit, Item, const K: usize> Volume<Item> for [Range<Unit>; K]
where
    Item: AsCoordinate<Unit, K>,
    Unit: PartialOrd,
{
    fn covers(&self, item: &Item) -> bool {
        let point: [&Unit; K] = item.as_coordinate();
        self.iter()
            .enumerate()
            .all(|(axis, range)| range.contains(&point[axis]))
    }
}

/*
impl<Unit, Item, const K: usize> Volume<Item> for &[Range<Unit>; K]
where
    Item: AsArray<Unit, K>,
    Unit: PartialOrd,
    [Range<Unit>; K]: Volume<Item>,
{
    fn covers(&self, item: &Item) -> bool {
        <[Range<Unit>; K]>::covers(self, item)
    }
}
*/

impl<Unit, Item, const K: usize> KdTreeNode<Unit, Item, K>
where
    Item: AsCoordinate<Unit, K>,
    Unit: Halveable + Clone,
    Range<Unit>: MidPoint<Unit>,
    [Range<Unit>; K]: Volume<Item>,
    [(); 1 << K]: Sized,
{
    fn new(ranges: [Range<Unit>; K], max_items: usize) -> Self {
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
                    let mut owned_items: Vec<Item> =
                        Vec::with_capacity(items.len());
                    *items = std::mem::replace(&mut owned_items, vec![]);
                    let _ = self.split();
                    owned_items.into_iter().for_each(|item| self.insert(item));
                }
            }
            Self::Parent { children, .. } => {
                for child in children.iter_mut() {
                    if child.get_ranges().covers(&item) {
                        child.insert(item);
                        break;
                    }
                }
            }
        };
    }

    fn insert_items(&mut self, items: Vec<Item>) {
        items.into_iter().for_each(|item| self.insert(item));
    }

    #[inline]
    fn get_ranges(&self) -> &[Range<Unit>; K] {
        match self {
            Self::Leaf { ranges, .. } => ranges,
            Self::Parent { ranges, .. } => ranges,
        }
    }

    // If [a, b) is the parent node's range along a certain axis,
    // then the child's range along that axis will be either
    // [a, c) or [c, b), where c = (b-a) / 2 is the midpoint.
    // As such, since there are K axes, a split will result in
    // 2^K distinct children. We can encode the unique
    // configuration of a particular child by using the 2^K
    // binary numbers from 0 to 2^K - 1. If the i:th bit is 1,
    // then the range along the i:th axis is [c, b), otherwise
    // the range is [a, c).
    fn split(&mut self) -> Result<(), KdTreeError> {
        // If the range has 0 volume, it makes no sense to split.
        if self.get_ranges().iter().any(|range| range.is_empty()) {
            return Err(KdTreeError::EmptyRegion);
        }
        let self_owned = unsafe { std::ptr::read(self) };
        match self_owned {
            // Splitting path
            Self::Leaf {
                ranges, max_items, ..
            } => {
                let children: [KdTreeNode<Unit, Item, K>; 1 << K] =
                    std::array::from_fn(|combination| {
                        let subranges: [Range<Unit>; K] =
                            std::array::from_fn(|i| {
                                let range = &ranges[i];
                                let mid_point = range.mid_point();
                                if combination & (1 << i) == 0 {
                                    range.start.clone()..mid_point
                                } else {
                                    mid_point..range.end.clone()
                                }
                            });
                        KdTreeNode::new(subranges, max_items)
                    });

                let new_self = Self::Parent {
                    ranges,
                    max_items,
                    children: Box::new(children),
                };
                //new_self.insert_items(items);
                //items.into_iter().for_each(|item| new_self.insert(item));
                unsafe {
                    std::ptr::write(self, new_self);
                };
                Ok(())
            }
            other => {
                unsafe {
                    std::ptr::write(self, other);
                };
                return Err(KdTreeError::NodeAlreadySplit);
            }
        }
    }
}

impl<Unit, Item, const K: usize> KdTree<Unit, Item, K>
where
    Item: AsCoordinate<Unit, K>,
    Unit: Halveable + Clone,
    Range<Unit>: MidPoint<Unit>,
    [Range<Unit>; K]: Volume<Item>,
    [(); 1 << K]: Sized,
{
    pub fn new(ranges: [Range<Unit>; K], max_items: usize) -> Self {
        let root: KdTreeNode<Unit, Item, K> = KdTreeNode::Leaf {
            ranges,
            items: Vec::<Item>::new(),
            max_items,
        };
        Self { root, max_items }
    }

    pub fn insert(&mut self, item: Item) {
        self.root.insert(item);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn print_type<T>(_: &T) {
        println!("{:?}", std::any::type_name::<T>());
    }

    #[test]
    fn bench() {
        use std::time::Instant;

        use rand::prelude::*;

        let mut rng = rand::rng();

        const MAX_LEAF_ITEMS: usize = 1000;
        const K: usize = 3;
        const N: usize = 10000;
        let bounds: [Range<f64>; K] = std::array::from_fn(|i| 0.0..1.0);
        let mut tree = KdTree::new(bounds, MAX_LEAF_ITEMS);
        let mut points: Vec<[f64; K]> = Vec::with_capacity(N);
        for _ in 0..N {
            let point: [f64; K] = rand::random();
            points.push(point);
        }

        let now = Instant::now();
        points.into_iter().for_each(|p| {
            tree.insert(p);
        });
        println!("{N} items took {} ms", now.elapsed().as_millis());
        //println!("{tree:#?}");
    }
}
